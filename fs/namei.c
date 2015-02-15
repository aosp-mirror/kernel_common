/*
 *  linux/fs/namei.c
 *
 *  Copyright (C) 1991, 1992  Linus Torvalds
 */



#include <linux/init.h>
#include <linux/export.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/namei.h>
#include <linux/pagemap.h>
#include <linux/fsnotify.h>
#include <linux/personality.h>
#include <linux/security.h>
#include <linux/ima.h>
#include <linux/syscalls.h>
#include <linux/mount.h>
#include <linux/audit.h>
#include <linux/capability.h>
#include <linux/file.h>
#include <linux/fcntl.h>
#include <linux/device_cgroup.h>
#include <linux/fs_struct.h>
#include <linux/posix_acl.h>
#include <asm/uaccess.h>
#include <trace/events/mmcio.h>

#include "internal.h"
#include "mount.h"





static int do_getname(const char __user *filename, char *page)
{
	int retval;
	unsigned long len = PATH_MAX;

	if (!segment_eq(get_fs(), KERNEL_DS)) {
		if ((unsigned long) filename >= TASK_SIZE)
			return -EFAULT;
		if (TASK_SIZE - (unsigned long) filename < PATH_MAX)
			len = TASK_SIZE - (unsigned long) filename;
	}

	retval = strncpy_from_user(page, filename, len);
	if (retval > 0) {
		if (retval < len)
			return 0;
		return -ENAMETOOLONG;
	} else if (!retval)
		retval = -ENOENT;
	return retval;
}

static char *getname_flags(const char __user *filename, int flags, int *empty)
{
	char *result = __getname();
	int retval;

	if (!result)
		return ERR_PTR(-ENOMEM);

	retval = do_getname(filename, result);
	if (retval < 0) {
		if (retval == -ENOENT && empty)
			*empty = 1;
		if (retval != -ENOENT || !(flags & LOOKUP_EMPTY)) {
			__putname(result);
			return ERR_PTR(retval);
		}
	}
	audit_getname(result);
	return result;
}

char *getname(const char __user * filename)
{
	return getname_flags(filename, 0, NULL);
}

#ifdef CONFIG_AUDITSYSCALL
void putname(const char *name)
{
	if (unlikely(!audit_dummy_context()))
		audit_putname(name);
	else
		__putname(name);
}
EXPORT_SYMBOL(putname);
#endif

static int check_acl(struct inode *inode, int mask)
{
#ifdef CONFIG_FS_POSIX_ACL
	struct posix_acl *acl;

	if (mask & MAY_NOT_BLOCK) {
		acl = get_cached_acl_rcu(inode, ACL_TYPE_ACCESS);
	        if (!acl)
	                return -EAGAIN;
		
		if (acl == ACL_NOT_CACHED)
			return -ECHILD;
	        return posix_acl_permission(inode, acl, mask & ~MAY_NOT_BLOCK);
	}

	acl = get_cached_acl(inode, ACL_TYPE_ACCESS);

	if (acl == ACL_NOT_CACHED) {
	        if (inode->i_op->get_acl) {
			acl = inode->i_op->get_acl(inode, ACL_TYPE_ACCESS);
			if (IS_ERR(acl))
				return PTR_ERR(acl);
		} else {
		        set_cached_acl(inode, ACL_TYPE_ACCESS, NULL);
		        return -EAGAIN;
		}
	}

	if (acl) {
	        int error = posix_acl_permission(inode, acl, mask);
	        posix_acl_release(acl);
	        return error;
	}
#endif

	return -EAGAIN;
}

static int acl_permission_check(struct inode *inode, int mask)
{
	unsigned int mode = inode->i_mode;

	if (current_user_ns() != inode_userns(inode))
		goto other_perms;

	if (likely(current_fsuid() == inode->i_uid))
		mode >>= 6;
	else {
		if (IS_POSIXACL(inode) && (mode & S_IRWXG)) {
			int error = check_acl(inode, mask);
			if (error != -EAGAIN)
				return error;
		}

		if (in_group_p(inode->i_gid))
			mode >>= 3;
	}

other_perms:
	if ((mask & ~mode & (MAY_READ | MAY_WRITE | MAY_EXEC)) == 0)
		return 0;
	return -EACCES;
}

int generic_permission(struct inode *inode, int mask)
{
	int ret;

	ret = acl_permission_check(inode, mask);
	if (ret != -EACCES)
		return ret;

	if (S_ISDIR(inode->i_mode)) {
		
		if (ns_capable(inode_userns(inode), CAP_DAC_OVERRIDE))
			return 0;
		if (!(mask & MAY_WRITE))
			if (ns_capable(inode_userns(inode), CAP_DAC_READ_SEARCH))
				return 0;
		return -EACCES;
	}
	if (!(mask & MAY_EXEC) || (inode->i_mode & S_IXUGO))
		if (ns_capable(inode_userns(inode), CAP_DAC_OVERRIDE))
			return 0;

	mask &= MAY_READ | MAY_WRITE | MAY_EXEC;
	if (mask == MAY_READ)
		if (ns_capable(inode_userns(inode), CAP_DAC_READ_SEARCH))
			return 0;

	return -EACCES;
}

static inline int do_inode_permission(struct inode *inode, int mask)
{
	if (unlikely(!(inode->i_opflags & IOP_FASTPERM))) {
		if (likely(inode->i_op->permission))
			return inode->i_op->permission(inode, mask);

		
		spin_lock(&inode->i_lock);
		inode->i_opflags |= IOP_FASTPERM;
		spin_unlock(&inode->i_lock);
	}
	return generic_permission(inode, mask);
}

int inode_permission(struct inode *inode, int mask)
{
	int retval;

	if (unlikely(mask & MAY_WRITE)) {
		umode_t mode = inode->i_mode;

		if (IS_RDONLY(inode) &&
		    (S_ISREG(mode) || S_ISDIR(mode) || S_ISLNK(mode)))
			return -EROFS;

		if (IS_IMMUTABLE(inode))
			return -EACCES;
	}

	retval = do_inode_permission(inode, mask);
	if (retval)
		return retval;

	retval = devcgroup_inode_permission(inode, mask);
	if (retval)
		return retval;

	return security_inode_permission(inode, mask);
}

void path_get(struct path *path)
{
	mntget(path->mnt);
	dget(path->dentry);
}
EXPORT_SYMBOL(path_get);

void path_put(struct path *path)
{
	dput(path->dentry);
	mntput(path->mnt);
}
EXPORT_SYMBOL(path_put);


static int unlazy_walk(struct nameidata *nd, struct dentry *dentry)
{
	struct fs_struct *fs = current->fs;
	struct dentry *parent = nd->path.dentry;
	int want_root = 0;

	BUG_ON(!(nd->flags & LOOKUP_RCU));
	if (nd->root.mnt && !(nd->flags & LOOKUP_ROOT)) {
		want_root = 1;
		spin_lock(&fs->lock);
		if (nd->root.mnt != fs->root.mnt ||
				nd->root.dentry != fs->root.dentry)
			goto err_root;
	}
	spin_lock(&parent->d_lock);
	if (!dentry) {
		if (!__d_rcu_to_refcount(parent, nd->seq))
			goto err_parent;
		BUG_ON(nd->inode != parent->d_inode);
	} else {
		if (dentry->d_parent != parent)
			goto err_parent;
		spin_lock_nested(&dentry->d_lock, DENTRY_D_LOCK_NESTED);
		if (!__d_rcu_to_refcount(dentry, nd->seq))
			goto err_child;
		BUG_ON(!IS_ROOT(dentry) && dentry->d_parent != parent);
		BUG_ON(!parent->d_count);
		parent->d_count++;
		spin_unlock(&dentry->d_lock);
	}
	spin_unlock(&parent->d_lock);
	if (want_root) {
		path_get(&nd->root);
		spin_unlock(&fs->lock);
	}
	mntget(nd->path.mnt);

	rcu_read_unlock();
	br_read_unlock(vfsmount_lock);
	nd->flags &= ~LOOKUP_RCU;
	return 0;

err_child:
	spin_unlock(&dentry->d_lock);
err_parent:
	spin_unlock(&parent->d_lock);
err_root:
	if (want_root)
		spin_unlock(&fs->lock);
	return -ECHILD;
}

void release_open_intent(struct nameidata *nd)
{
	struct file *file = nd->intent.open.file;

	if (file && !IS_ERR(file)) {
		if (file->f_path.dentry == NULL)
			put_filp(file);
		else
			fput(file);
	}
}

static inline int d_revalidate(struct dentry *dentry, struct nameidata *nd)
{
	return dentry->d_op->d_revalidate(dentry, nd);
}

static int complete_walk(struct nameidata *nd)
{
	struct dentry *dentry = nd->path.dentry;
	int status;

	if (nd->flags & LOOKUP_RCU) {
		nd->flags &= ~LOOKUP_RCU;
		if (!(nd->flags & LOOKUP_ROOT))
			nd->root.mnt = NULL;
		spin_lock(&dentry->d_lock);
		if (unlikely(!__d_rcu_to_refcount(dentry, nd->seq))) {
			spin_unlock(&dentry->d_lock);
			rcu_read_unlock();
			br_read_unlock(vfsmount_lock);
			return -ECHILD;
		}
		BUG_ON(nd->inode != dentry->d_inode);
		spin_unlock(&dentry->d_lock);
		mntget(nd->path.mnt);
		rcu_read_unlock();
		br_read_unlock(vfsmount_lock);
	}

	if (likely(!(nd->flags & LOOKUP_JUMPED)))
		return 0;

	if (likely(!(dentry->d_flags & DCACHE_OP_REVALIDATE)))
		return 0;

	if (likely(!(dentry->d_sb->s_type->fs_flags & FS_REVAL_DOT)))
		return 0;

	
	status = d_revalidate(dentry, nd);
	if (status > 0)
		return 0;

	if (!status)
		status = -ESTALE;

	path_put(&nd->path);
	return status;
}

static __always_inline void set_root(struct nameidata *nd)
{
	if (!nd->root.mnt)
		get_fs_root(current->fs, &nd->root);
}

static int link_path_walk(const char *, struct nameidata *);

static __always_inline void set_root_rcu(struct nameidata *nd)
{
	if (!nd->root.mnt) {
		struct fs_struct *fs = current->fs;
		unsigned seq;

		do {
			seq = read_seqcount_begin(&fs->seq);
			nd->root = fs->root;
			nd->seq = __read_seqcount_begin(&nd->root.dentry->d_seq);
		} while (read_seqcount_retry(&fs->seq, seq));
	}
}

static __always_inline int __vfs_follow_link(struct nameidata *nd, const char *link)
{
	int ret;

	if (IS_ERR(link))
		goto fail;

	if (*link == '/') {
		set_root(nd);
		path_put(&nd->path);
		nd->path = nd->root;
		path_get(&nd->root);
		nd->flags |= LOOKUP_JUMPED;
	}
	nd->inode = nd->path.dentry->d_inode;

	ret = link_path_walk(link, nd);
	return ret;
fail:
	path_put(&nd->path);
	return PTR_ERR(link);
}

static void path_put_conditional(struct path *path, struct nameidata *nd)
{
	dput(path->dentry);
	if (path->mnt != nd->path.mnt)
		mntput(path->mnt);
}

static inline void path_to_nameidata(const struct path *path,
					struct nameidata *nd)
{
	if (!(nd->flags & LOOKUP_RCU)) {
		dput(nd->path.dentry);
		if (nd->path.mnt != path->mnt)
			mntput(nd->path.mnt);
	}
	nd->path.mnt = path->mnt;
	nd->path.dentry = path->dentry;
}

static inline void put_link(struct nameidata *nd, struct path *link, void *cookie)
{
	struct inode *inode = link->dentry->d_inode;
	if (!IS_ERR(cookie) && inode->i_op->put_link)
		inode->i_op->put_link(link->dentry, nd, cookie);
	path_put(link);
}

static __always_inline int
follow_link(struct path *link, struct nameidata *nd, void **p)
{
	int error;
	struct dentry *dentry = link->dentry;

	BUG_ON(nd->flags & LOOKUP_RCU);

	if (link->mnt == nd->path.mnt)
		mntget(link->mnt);

	if (unlikely(current->total_link_count >= 40)) {
		*p = ERR_PTR(-ELOOP); 
		path_put(&nd->path);
		return -ELOOP;
	}
	cond_resched();
	current->total_link_count++;

	touch_atime(link);
	nd_set_link(nd, NULL);

	error = security_inode_follow_link(link->dentry, nd);
	if (error) {
		*p = ERR_PTR(error); 
		path_put(&nd->path);
		return error;
	}

	nd->last_type = LAST_BIND;
	*p = dentry->d_inode->i_op->follow_link(dentry, nd);
	error = PTR_ERR(*p);
	if (!IS_ERR(*p)) {
		char *s = nd_get_link(nd);
		error = 0;
		if (s)
			error = __vfs_follow_link(nd, s);
		else if (nd->last_type == LAST_BIND) {
			nd->flags |= LOOKUP_JUMPED;
			nd->inode = nd->path.dentry->d_inode;
			if (nd->inode->i_op->follow_link) {
				
				path_put(&nd->path);
				error = -ELOOP;
			}
		}
	}
	return error;
}

static int follow_up_rcu(struct path *path)
{
	struct mount *mnt = real_mount(path->mnt);
	struct mount *parent;
	struct dentry *mountpoint;

	parent = mnt->mnt_parent;
	if (&parent->mnt == path->mnt)
		return 0;
	mountpoint = mnt->mnt_mountpoint;
	path->dentry = mountpoint;
	path->mnt = &parent->mnt;
	return 1;
}

int follow_up(struct path *path)
{
	struct mount *mnt = real_mount(path->mnt);
	struct mount *parent;
	struct dentry *mountpoint;

	br_read_lock(vfsmount_lock);
	parent = mnt->mnt_parent;
	if (&parent->mnt == path->mnt) {
		br_read_unlock(vfsmount_lock);
		return 0;
	}
	mntget(&parent->mnt);
	mountpoint = dget(mnt->mnt_mountpoint);
	br_read_unlock(vfsmount_lock);
	dput(path->dentry);
	path->dentry = mountpoint;
	mntput(path->mnt);
	path->mnt = &parent->mnt;
	return 1;
}

static int follow_automount(struct path *path, unsigned flags,
			    bool *need_mntput)
{
	struct vfsmount *mnt;
	int err;

	if (!path->dentry->d_op || !path->dentry->d_op->d_automount)
		return -EREMOTE;

	if (!(flags & (LOOKUP_PARENT | LOOKUP_DIRECTORY |
		     LOOKUP_OPEN | LOOKUP_CREATE | LOOKUP_AUTOMOUNT)) &&
	    path->dentry->d_inode)
		return -EISDIR;

	current->total_link_count++;
	if (current->total_link_count >= 40)
		return -ELOOP;

	mnt = path->dentry->d_op->d_automount(path);
	if (IS_ERR(mnt)) {
		if (PTR_ERR(mnt) == -EISDIR && (flags & LOOKUP_PARENT))
			return -EREMOTE;
		return PTR_ERR(mnt);
	}

	if (!mnt) 
		return 0;

	if (!*need_mntput) {
		
		mntget(path->mnt);
		*need_mntput = true;
	}
	err = finish_automount(mnt, path);

	switch (err) {
	case -EBUSY:
		
		return 0;
	case 0:
		path_put(path);
		path->mnt = mnt;
		path->dentry = dget(mnt->mnt_root);
		return 0;
	default:
		return err;
	}

}

static int follow_managed(struct path *path, unsigned flags)
{
	struct vfsmount *mnt = path->mnt; 
	unsigned managed;
	bool need_mntput = false;
	int ret = 0;

	while (managed = ACCESS_ONCE(path->dentry->d_flags),
	       managed &= DCACHE_MANAGED_DENTRY,
	       unlikely(managed != 0)) {
		if (managed & DCACHE_MANAGE_TRANSIT) {
			BUG_ON(!path->dentry->d_op);
			BUG_ON(!path->dentry->d_op->d_manage);
			ret = path->dentry->d_op->d_manage(path->dentry, false);
			if (ret < 0)
				break;
		}

		
		if (managed & DCACHE_MOUNTED) {
			struct vfsmount *mounted = lookup_mnt(path);
			if (mounted) {
				dput(path->dentry);
				if (need_mntput)
					mntput(path->mnt);
				path->mnt = mounted;
				path->dentry = dget(mounted->mnt_root);
				need_mntput = true;
				continue;
			}

		}

		
		if (managed & DCACHE_NEED_AUTOMOUNT) {
			ret = follow_automount(path, flags, &need_mntput);
			if (ret < 0)
				break;
			continue;
		}

		
		break;
	}

	if (need_mntput && path->mnt == mnt)
		mntput(path->mnt);
	if (ret == -EISDIR)
		ret = 0;
	return ret < 0 ? ret : need_mntput;
}

int follow_down_one(struct path *path)
{
	struct vfsmount *mounted;

	mounted = lookup_mnt(path);
	if (mounted) {
		dput(path->dentry);
		mntput(path->mnt);
		path->mnt = mounted;
		path->dentry = dget(mounted->mnt_root);
		return 1;
	}
	return 0;
}

static inline bool managed_dentry_might_block(struct dentry *dentry)
{
	return (dentry->d_flags & DCACHE_MANAGE_TRANSIT &&
		dentry->d_op->d_manage(dentry, true) < 0);
}

static bool __follow_mount_rcu(struct nameidata *nd, struct path *path,
			       struct inode **inode)
{
	for (;;) {
		struct mount *mounted;
		if (unlikely(managed_dentry_might_block(path->dentry)))
			return false;

		if (!d_mountpoint(path->dentry))
			break;

		mounted = __lookup_mnt(path->mnt, path->dentry, 1);
		if (!mounted)
			break;
		path->mnt = &mounted->mnt;
		path->dentry = mounted->mnt.mnt_root;
		nd->flags |= LOOKUP_JUMPED;
		nd->seq = read_seqcount_begin(&path->dentry->d_seq);
		*inode = path->dentry->d_inode;
	}
	return true;
}

static void follow_mount_rcu(struct nameidata *nd)
{
	while (d_mountpoint(nd->path.dentry)) {
		struct mount *mounted;
		mounted = __lookup_mnt(nd->path.mnt, nd->path.dentry, 1);
		if (!mounted)
			break;
		nd->path.mnt = &mounted->mnt;
		nd->path.dentry = mounted->mnt.mnt_root;
		nd->seq = read_seqcount_begin(&nd->path.dentry->d_seq);
	}
}

static int follow_dotdot_rcu(struct nameidata *nd)
{
	set_root_rcu(nd);

	while (1) {
		if (nd->path.dentry == nd->root.dentry &&
		    nd->path.mnt == nd->root.mnt) {
			break;
		}
		if (nd->path.dentry != nd->path.mnt->mnt_root) {
			struct dentry *old = nd->path.dentry;
			struct dentry *parent = old->d_parent;
			unsigned seq;

			seq = read_seqcount_begin(&parent->d_seq);
			if (read_seqcount_retry(&old->d_seq, nd->seq))
				goto failed;
			nd->path.dentry = parent;
			nd->seq = seq;
			break;
		}
		if (!follow_up_rcu(&nd->path))
			break;
		nd->seq = read_seqcount_begin(&nd->path.dentry->d_seq);
	}
	follow_mount_rcu(nd);
	nd->inode = nd->path.dentry->d_inode;
	return 0;

failed:
	nd->flags &= ~LOOKUP_RCU;
	if (!(nd->flags & LOOKUP_ROOT))
		nd->root.mnt = NULL;
	rcu_read_unlock();
	br_read_unlock(vfsmount_lock);
	return -ECHILD;
}

int follow_down(struct path *path)
{
	unsigned managed;
	int ret;

	while (managed = ACCESS_ONCE(path->dentry->d_flags),
	       unlikely(managed & DCACHE_MANAGED_DENTRY)) {
		if (managed & DCACHE_MANAGE_TRANSIT) {
			BUG_ON(!path->dentry->d_op);
			BUG_ON(!path->dentry->d_op->d_manage);
			ret = path->dentry->d_op->d_manage(
				path->dentry, false);
			if (ret < 0)
				return ret == -EISDIR ? 0 : ret;
		}

		
		if (managed & DCACHE_MOUNTED) {
			struct vfsmount *mounted = lookup_mnt(path);
			if (!mounted)
				break;
			dput(path->dentry);
			mntput(path->mnt);
			path->mnt = mounted;
			path->dentry = dget(mounted->mnt_root);
			continue;
		}

		
		break;
	}
	return 0;
}

static void follow_mount(struct path *path)
{
	while (d_mountpoint(path->dentry)) {
		struct vfsmount *mounted = lookup_mnt(path);
		if (!mounted)
			break;
		dput(path->dentry);
		mntput(path->mnt);
		path->mnt = mounted;
		path->dentry = dget(mounted->mnt_root);
	}
}

static void follow_dotdot(struct nameidata *nd)
{
	set_root(nd);

	while(1) {
		struct dentry *old = nd->path.dentry;

		if (nd->path.dentry == nd->root.dentry &&
		    nd->path.mnt == nd->root.mnt) {
			break;
		}
		if (nd->path.dentry != nd->path.mnt->mnt_root) {
			
			nd->path.dentry = dget_parent(nd->path.dentry);
			dput(old);
			break;
		}
		if (!follow_up(&nd->path))
			break;
	}
	follow_mount(&nd->path);
	nd->inode = nd->path.dentry->d_inode;
}

static struct dentry *lookup_dcache(struct qstr *name, struct dentry *dir,
				    struct nameidata *nd, bool *need_lookup)
{
	struct dentry *dentry;
	int error;

	*need_lookup = false;
	dentry = d_lookup(dir, name);
	if (dentry) {
		if (d_need_lookup(dentry)) {
			*need_lookup = true;
		} else if (dentry->d_flags & DCACHE_OP_REVALIDATE) {
			error = d_revalidate(dentry, nd);
			if (unlikely(error <= 0)) {
				if (error < 0) {
					dput(dentry);
					return ERR_PTR(error);
				} else if (!d_invalidate(dentry)) {
					dput(dentry);
					dentry = NULL;
				}
			}
		}
	}

	if (!dentry) {
		dentry = d_alloc(dir, name);
		if (unlikely(!dentry))
			return ERR_PTR(-ENOMEM);

		*need_lookup = true;
	}
	return dentry;
}

static struct dentry *lookup_real(struct inode *dir, struct dentry *dentry,
				  struct nameidata *nd)
{
	struct dentry *old;

	
	if (unlikely(IS_DEADDIR(dir))) {
		dput(dentry);
		return ERR_PTR(-ENOENT);
	}

	old = dir->i_op->lookup(dir, dentry, nd);
	if (unlikely(old)) {
		dput(dentry);
		dentry = old;
	}
	return dentry;
}

static struct dentry *__lookup_hash(struct qstr *name,
		struct dentry *base, struct nameidata *nd)
{
	bool need_lookup;
	struct dentry *dentry;

	dentry = lookup_dcache(name, base, nd, &need_lookup);
	if (!need_lookup)
		return dentry;

	return lookup_real(base->d_inode, dentry, nd);
}

static int do_lookup(struct nameidata *nd, struct qstr *name,
			struct path *path, struct inode **inode)
{
	struct vfsmount *mnt = nd->path.mnt;
	struct dentry *dentry, *parent = nd->path.dentry;
	int need_reval = 1;
	int status = 1;
	int err;

	if (nd->flags & LOOKUP_RCU) {
		unsigned seq;
		*inode = nd->inode;
		dentry = __d_lookup_rcu(parent, name, &seq, inode);
		if (!dentry)
			goto unlazy;

		
		if (__read_seqcount_retry(&parent->d_seq, nd->seq))
			return -ECHILD;
		nd->seq = seq;

		if (unlikely(d_need_lookup(dentry)))
			goto unlazy;
		if (unlikely(dentry->d_flags & DCACHE_OP_REVALIDATE)) {
			status = d_revalidate(dentry, nd);
			if (unlikely(status <= 0)) {
				if (status != -ECHILD)
					need_reval = 0;
				goto unlazy;
			}
		}
		path->mnt = mnt;
		path->dentry = dentry;
		if (unlikely(!__follow_mount_rcu(nd, path, inode)))
			goto unlazy;
		if (unlikely(path->dentry->d_flags & DCACHE_NEED_AUTOMOUNT))
			goto unlazy;
		return 0;
unlazy:
		if (unlazy_walk(nd, dentry))
			return -ECHILD;
	} else {
		dentry = __d_lookup(parent, name);
	}

	if (unlikely(!dentry))
		goto need_lookup;

	if (unlikely(d_need_lookup(dentry))) {
		dput(dentry);
		goto need_lookup;
	}

	if (unlikely(dentry->d_flags & DCACHE_OP_REVALIDATE) && need_reval)
		status = d_revalidate(dentry, nd);
	if (unlikely(status <= 0)) {
		if (status < 0) {
			dput(dentry);
			return status;
		}
		if (!d_invalidate(dentry)) {
			dput(dentry);
			goto need_lookup;
		}
	}
done:
	path->mnt = mnt;
	path->dentry = dentry;
	err = follow_managed(path, nd->flags);
	if (unlikely(err < 0)) {
		path_put_conditional(path, nd);
		return err;
	}
	if (err)
		nd->flags |= LOOKUP_JUMPED;
	*inode = path->dentry->d_inode;
	return 0;

need_lookup:
	BUG_ON(nd->inode != parent->d_inode);

	mutex_lock(&parent->d_inode->i_mutex);
	dentry = __lookup_hash(name, parent, nd);
	mutex_unlock(&parent->d_inode->i_mutex);
	if (IS_ERR(dentry))
		return PTR_ERR(dentry);
	goto done;
}

static inline int may_lookup(struct nameidata *nd)
{
	if (nd->flags & LOOKUP_RCU) {
		int err = inode_permission(nd->inode, MAY_EXEC|MAY_NOT_BLOCK);
		if (err != -ECHILD)
			return err;
		if (unlazy_walk(nd, NULL))
			return -ECHILD;
	}
	return inode_permission(nd->inode, MAY_EXEC);
}

static inline int handle_dots(struct nameidata *nd, int type)
{
	if (type == LAST_DOTDOT) {
		if (nd->flags & LOOKUP_RCU) {
			if (follow_dotdot_rcu(nd))
				return -ECHILD;
		} else
			follow_dotdot(nd);
	}
	return 0;
}

static void terminate_walk(struct nameidata *nd)
{
	if (!(nd->flags & LOOKUP_RCU)) {
		path_put(&nd->path);
	} else {
		nd->flags &= ~LOOKUP_RCU;
		if (!(nd->flags & LOOKUP_ROOT))
			nd->root.mnt = NULL;
		rcu_read_unlock();
		br_read_unlock(vfsmount_lock);
	}
}

static inline int should_follow_link(struct inode *inode, int follow)
{
	if (unlikely(!(inode->i_opflags & IOP_NOFOLLOW))) {
		if (likely(inode->i_op->follow_link))
			return follow;

		
		spin_lock(&inode->i_lock);
		inode->i_opflags |= IOP_NOFOLLOW;
		spin_unlock(&inode->i_lock);
	}
	return 0;
}

static inline int walk_component(struct nameidata *nd, struct path *path,
		struct qstr *name, int type, int follow)
{
	struct inode *inode;
	int err;
	if (unlikely(type != LAST_NORM))
		return handle_dots(nd, type);
	err = do_lookup(nd, name, path, &inode);
	if (unlikely(err)) {
		terminate_walk(nd);
		return err;
	}
	if (!inode) {
		path_to_nameidata(path, nd);
		terminate_walk(nd);
		return -ENOENT;
	}
	if (should_follow_link(inode, follow)) {
		if (nd->flags & LOOKUP_RCU) {
			if (unlikely(unlazy_walk(nd, path->dentry))) {
				terminate_walk(nd);
				return -ECHILD;
			}
		}
		BUG_ON(inode != path->dentry->d_inode);
		return 1;
	}
	path_to_nameidata(path, nd);
	nd->inode = inode;
	return 0;
}

static inline int nested_symlink(struct path *path, struct nameidata *nd)
{
	int res;

	if (unlikely(current->link_count >= MAX_NESTED_LINKS)) {
		path_put_conditional(path, nd);
		path_put(&nd->path);
		return -ELOOP;
	}
	BUG_ON(nd->depth >= MAX_NESTED_LINKS);

	nd->depth++;
	current->link_count++;

	do {
		struct path link = *path;
		void *cookie;

		res = follow_link(&link, nd, &cookie);
		if (!res)
			res = walk_component(nd, path, &nd->last,
					     nd->last_type, LOOKUP_FOLLOW);
		put_link(nd, &link, cookie);
	} while (res > 0);

	current->link_count--;
	nd->depth--;
	return res;
}

static inline int can_lookup(struct inode *inode)
{
	if (likely(inode->i_opflags & IOP_LOOKUP))
		return 1;
	if (likely(!inode->i_op->lookup))
		return 0;

	
	spin_lock(&inode->i_lock);
	inode->i_opflags |= IOP_LOOKUP;
	spin_unlock(&inode->i_lock);
	return 1;
}

#ifdef CONFIG_DCACHE_WORD_ACCESS

#include <asm/word-at-a-time.h>

#ifdef CONFIG_64BIT

static inline unsigned int fold_hash(unsigned long hash)
{
	hash += hash >> (8*sizeof(int));
	return hash;
}

#else	

#define fold_hash(x) (x)

#endif

unsigned int full_name_hash(const unsigned char *name, unsigned int len)
{
	unsigned long a, mask;
	unsigned long hash = 0;

	for (;;) {
		a = load_unaligned_zeropad(name);
		if (len < sizeof(unsigned long))
			break;
		hash += a;
		hash *= 9;
		name += sizeof(unsigned long);
		len -= sizeof(unsigned long);
		if (!len)
			goto done;
	}
	mask = ~(~0ul << len*8);
	hash += mask & a;
done:
	return fold_hash(hash);
}
EXPORT_SYMBOL(full_name_hash);

static inline unsigned long hash_name(const char *name, unsigned int *hashp)
{
	unsigned long a, mask, hash, len;

	hash = a = 0;
	len = -sizeof(unsigned long);
	do {
		hash = (hash + a) * 9;
		len += sizeof(unsigned long);
		a = load_unaligned_zeropad(name+len);
		
		mask = has_zero(a) | has_zero(a ^ REPEAT_BYTE('/'));
	} while (!mask);

	
	mask = (mask - 1) & ~mask;
	mask >>= 7;
	hash += a & mask;
	*hashp = fold_hash(hash);

	return len + count_masked_bytes(mask);
}

#else

unsigned int full_name_hash(const unsigned char *name, unsigned int len)
{
	unsigned long hash = init_name_hash();
	while (len--)
		hash = partial_name_hash(*name++, hash);
	return end_name_hash(hash);
}
EXPORT_SYMBOL(full_name_hash);

static inline unsigned long hash_name(const char *name, unsigned int *hashp)
{
	unsigned long hash = init_name_hash();
	unsigned long len = 0, c;

	c = (unsigned char)*name;
	do {
		len++;
		hash = partial_name_hash(c, hash);
		c = (unsigned char)name[len];
	} while (c && c != '/');
	*hashp = end_name_hash(hash);
	return len;
}

#endif

static int link_path_walk(const char *name, struct nameidata *nd)
{
	struct path next;
	int err;
	
	while (*name=='/')
		name++;
	if (!*name)
		return 0;

	
	for(;;) {
		struct qstr this;
		long len;
		int type;

		err = may_lookup(nd);
 		if (err)
			break;

		len = hash_name(name, &this.hash);
		this.name = name;
		this.len = len;

		type = LAST_NORM;
		if (name[0] == '.') switch (len) {
			case 2:
				if (name[1] == '.') {
					type = LAST_DOTDOT;
					nd->flags |= LOOKUP_JUMPED;
				}
				break;
			case 1:
				type = LAST_DOT;
		}
		if (likely(type == LAST_NORM)) {
			struct dentry *parent = nd->path.dentry;
			nd->flags &= ~LOOKUP_JUMPED;
			if (unlikely(parent->d_flags & DCACHE_OP_HASH)) {
				err = parent->d_op->d_hash(parent, nd->inode,
							   &this);
				if (err < 0)
					break;
			}
		}

		if (!name[len])
			goto last_component;
		do {
			len++;
		} while (unlikely(name[len] == '/'));
		if (!name[len])
			goto last_component;
		name += len;

		err = walk_component(nd, &next, &this, type, LOOKUP_FOLLOW);
		if (err < 0)
			return err;

		if (err) {
			err = nested_symlink(&next, nd);
			if (err)
				return err;
		}
		if (can_lookup(nd->inode))
			continue;
		err = -ENOTDIR; 
		break;
		

last_component:
		nd->last = this;
		nd->last_type = type;
		return 0;
	}
	terminate_walk(nd);
	return err;
}

static int path_init(int dfd, const char *name, unsigned int flags,
		     struct nameidata *nd, struct file **fp)
{
	int retval = 0;
	int fput_needed;
	struct file *file;

	nd->last_type = LAST_ROOT; 
	nd->flags = flags | LOOKUP_JUMPED;
	nd->depth = 0;
	if (flags & LOOKUP_ROOT) {
		struct inode *inode = nd->root.dentry->d_inode;
		if (*name) {
			if (!inode->i_op->lookup)
				return -ENOTDIR;
			retval = inode_permission(inode, MAY_EXEC);
			if (retval)
				return retval;
		}
		nd->path = nd->root;
		nd->inode = inode;
		if (flags & LOOKUP_RCU) {
			br_read_lock(vfsmount_lock);
			rcu_read_lock();
			nd->seq = __read_seqcount_begin(&nd->path.dentry->d_seq);
		} else {
			path_get(&nd->path);
		}
		return 0;
	}

	nd->root.mnt = NULL;

	if (*name=='/') {
		if (flags & LOOKUP_RCU) {
			br_read_lock(vfsmount_lock);
			rcu_read_lock();
			set_root_rcu(nd);
		} else {
			set_root(nd);
			path_get(&nd->root);
		}
		nd->path = nd->root;
	} else if (dfd == AT_FDCWD) {
		if (flags & LOOKUP_RCU) {
			struct fs_struct *fs = current->fs;
			unsigned seq;

			br_read_lock(vfsmount_lock);
			rcu_read_lock();

			do {
				seq = read_seqcount_begin(&fs->seq);
				nd->path = fs->pwd;
				nd->seq = __read_seqcount_begin(&nd->path.dentry->d_seq);
			} while (read_seqcount_retry(&fs->seq, seq));
		} else {
			get_fs_pwd(current->fs, &nd->path);
		}
	} else {
		struct dentry *dentry;

		file = fget_raw_light(dfd, &fput_needed);
		retval = -EBADF;
		if (!file)
			goto out_fail;

		dentry = file->f_path.dentry;

		if (*name) {
			retval = -ENOTDIR;
			if (!S_ISDIR(dentry->d_inode->i_mode))
				goto fput_fail;

			retval = inode_permission(dentry->d_inode, MAY_EXEC);
			if (retval)
				goto fput_fail;
		}

		nd->path = file->f_path;
		if (flags & LOOKUP_RCU) {
			if (fput_needed)
				*fp = file;
			nd->seq = __read_seqcount_begin(&nd->path.dentry->d_seq);
			br_read_lock(vfsmount_lock);
			rcu_read_lock();
		} else {
			path_get(&file->f_path);
			fput_light(file, fput_needed);
		}
	}

	nd->inode = nd->path.dentry->d_inode;
	return 0;

fput_fail:
	fput_light(file, fput_needed);
out_fail:
	return retval;
}

static inline int lookup_last(struct nameidata *nd, struct path *path)
{
	if (nd->last_type == LAST_NORM && nd->last.name[nd->last.len])
		nd->flags |= LOOKUP_FOLLOW | LOOKUP_DIRECTORY;

	nd->flags &= ~LOOKUP_PARENT;
	return walk_component(nd, path, &nd->last, nd->last_type,
					nd->flags & LOOKUP_FOLLOW);
}

static int path_lookupat(int dfd, const char *name,
				unsigned int flags, struct nameidata *nd)
{
	struct file *base = NULL;
	struct path path;
	int err;

	err = path_init(dfd, name, flags | LOOKUP_PARENT, nd, &base);

	if (unlikely(err))
		return err;

	current->total_link_count = 0;
	err = link_path_walk(name, nd);

	if (!err && !(flags & LOOKUP_PARENT)) {
		err = lookup_last(nd, &path);
		while (err > 0) {
			void *cookie;
			struct path link = path;
			nd->flags |= LOOKUP_PARENT;
			err = follow_link(&link, nd, &cookie);
			if (!err)
				err = lookup_last(nd, &path);
			put_link(nd, &link, cookie);
		}
	}

	if (!err)
		err = complete_walk(nd);

	if (!err && nd->flags & LOOKUP_DIRECTORY) {
		if (!nd->inode->i_op->lookup) {
			path_put(&nd->path);
			err = -ENOTDIR;
		}
	}

	if (base)
		fput(base);

	if (nd->root.mnt && !(nd->flags & LOOKUP_ROOT)) {
		path_put(&nd->root);
		nd->root.mnt = NULL;
	}
	return err;
}

static int do_path_lookup(int dfd, const char *name,
				unsigned int flags, struct nameidata *nd)
{
	int retval = path_lookupat(dfd, name, flags | LOOKUP_RCU, nd);
	if (unlikely(retval == -ECHILD))
		retval = path_lookupat(dfd, name, flags, nd);
	if (unlikely(retval == -ESTALE))
		retval = path_lookupat(dfd, name, flags | LOOKUP_REVAL, nd);

	if (likely(!retval)) {
		if (unlikely(!audit_dummy_context())) {
			if (nd->path.dentry && nd->inode)
				audit_inode(name, nd->path.dentry);
		}
	}
	return retval;
}

int kern_path_parent(const char *name, struct nameidata *nd)
{
	return do_path_lookup(AT_FDCWD, name, LOOKUP_PARENT, nd);
}

int kern_path(const char *name, unsigned int flags, struct path *path)
{
	struct nameidata nd;
	int res = do_path_lookup(AT_FDCWD, name, flags, &nd);
	if (!res)
		*path = nd.path;
	return res;
}

int vfs_path_lookup(struct dentry *dentry, struct vfsmount *mnt,
		    const char *name, unsigned int flags,
		    struct path *path)
{
	struct nameidata nd;
	int err;
	nd.root.dentry = dentry;
	nd.root.mnt = mnt;
	BUG_ON(flags & LOOKUP_PARENT);
	
	err = do_path_lookup(AT_FDCWD, name, flags | LOOKUP_ROOT, &nd);
	if (!err)
		*path = nd.path;
	return err;
}

static struct dentry *lookup_hash(struct nameidata *nd)
{
	return __lookup_hash(&nd->last, nd->path.dentry, nd);
}

struct dentry *lookup_one_len(const char *name, struct dentry *base, int len)
{
	struct qstr this;
	unsigned int c;
	int err;

	WARN_ON_ONCE(!mutex_is_locked(&base->d_inode->i_mutex));

	this.name = name;
	this.len = len;
	this.hash = full_name_hash(name, len);
	if (!len)
		return ERR_PTR(-EACCES);

	while (len--) {
		c = *(const unsigned char *)name++;
		if (c == '/' || c == '\0')
			return ERR_PTR(-EACCES);
	}
	if (base->d_flags & DCACHE_OP_HASH) {
		int err = base->d_op->d_hash(base, base->d_inode, &this);
		if (err < 0)
			return ERR_PTR(err);
	}

	err = inode_permission(base->d_inode, MAY_EXEC);
	if (err)
		return ERR_PTR(err);

	return __lookup_hash(&this, base, NULL);
}

int user_path_at_empty(int dfd, const char __user *name, unsigned flags,
		 struct path *path, int *empty)
{
	struct nameidata nd;
	char *tmp = getname_flags(name, flags, empty);
	int err = PTR_ERR(tmp);
	if (!IS_ERR(tmp)) {

		BUG_ON(flags & LOOKUP_PARENT);

		err = do_path_lookup(dfd, tmp, flags, &nd);
		putname(tmp);
		if (!err)
			*path = nd.path;
	}
	return err;
}

int user_path_at(int dfd, const char __user *name, unsigned flags,
		 struct path *path)
{
	return user_path_at_empty(dfd, name, flags, path, NULL);
}

static int user_path_parent(int dfd, const char __user *path,
			struct nameidata *nd, char **name)
{
	char *s = getname(path);
	int error;

	if (IS_ERR(s))
		return PTR_ERR(s);

	error = do_path_lookup(dfd, s, LOOKUP_PARENT, nd);
	if (error)
		putname(s);
	else
		*name = s;

	return error;
}

static inline int check_sticky(struct inode *dir, struct inode *inode)
{
	uid_t fsuid = current_fsuid();

	if (!(dir->i_mode & S_ISVTX))
		return 0;
	if (current_user_ns() != inode_userns(inode))
		goto other_userns;
	if (inode->i_uid == fsuid)
		return 0;
	if (dir->i_uid == fsuid)
		return 0;

other_userns:
	return !ns_capable(inode_userns(inode), CAP_FOWNER);
}

static int may_delete(struct inode *dir,struct dentry *victim,int isdir)
{
	int error;

	if (!victim->d_inode)
		return -ENOENT;

	BUG_ON(victim->d_parent->d_inode != dir);
	audit_inode_child(victim, dir);

	error = inode_permission(dir, MAY_WRITE | MAY_EXEC);
	if (error)
		return error;
	if (IS_APPEND(dir))
		return -EPERM;
	if (check_sticky(dir, victim->d_inode)||IS_APPEND(victim->d_inode)||
	    IS_IMMUTABLE(victim->d_inode) || IS_SWAPFILE(victim->d_inode))
		return -EPERM;
	if (isdir) {
		if (!S_ISDIR(victim->d_inode->i_mode))
			return -ENOTDIR;
		if (IS_ROOT(victim))
			return -EBUSY;
	} else if (S_ISDIR(victim->d_inode->i_mode))
		return -EISDIR;
	if (IS_DEADDIR(dir))
		return -ENOENT;
	if (victim->d_flags & DCACHE_NFSFS_RENAMED)
		return -EBUSY;
	return 0;
}

static inline int may_create(struct inode *dir, struct dentry *child)
{
	if (child->d_inode)
		return -EEXIST;
	if (IS_DEADDIR(dir))
		return -ENOENT;
	return inode_permission(dir, MAY_WRITE | MAY_EXEC);
}

struct dentry *lock_rename(struct dentry *p1, struct dentry *p2)
{
	struct dentry *p;

	if (p1 == p2) {
		mutex_lock_nested(&p1->d_inode->i_mutex, I_MUTEX_PARENT);
		return NULL;
	}

	mutex_lock(&p1->d_inode->i_sb->s_vfs_rename_mutex);

	p = d_ancestor(p2, p1);
	if (p) {
		mutex_lock_nested(&p2->d_inode->i_mutex, I_MUTEX_PARENT);
		mutex_lock_nested(&p1->d_inode->i_mutex, I_MUTEX_CHILD);
		return p;
	}

	p = d_ancestor(p1, p2);
	if (p) {
		mutex_lock_nested(&p1->d_inode->i_mutex, I_MUTEX_PARENT);
		mutex_lock_nested(&p2->d_inode->i_mutex, I_MUTEX_CHILD);
		return p;
	}

	mutex_lock_nested(&p1->d_inode->i_mutex, I_MUTEX_PARENT);
	mutex_lock_nested(&p2->d_inode->i_mutex, I_MUTEX_CHILD);
	return NULL;
}

void unlock_rename(struct dentry *p1, struct dentry *p2)
{
	mutex_unlock(&p1->d_inode->i_mutex);
	if (p1 != p2) {
		mutex_unlock(&p2->d_inode->i_mutex);
		mutex_unlock(&p1->d_inode->i_sb->s_vfs_rename_mutex);
	}
}

int vfs_create(struct inode *dir, struct dentry *dentry, umode_t mode,
		struct nameidata *nd)
{
	int error = may_create(dir, dentry);

	if (error)
		return error;

	if (!dir->i_op->create)
		return -EACCES;	
	mode &= S_IALLUGO;
	mode |= S_IFREG;
	error = security_inode_create(dir, dentry, mode);
	if (error)
		return error;
	error = dir->i_op->create(dir, dentry, mode, nd);
	if (error)
		return error;

	error = security_inode_post_create(dir, dentry, mode);
	if (error)
		return error;

	if (!error)
		fsnotify_create(dir, dentry);
	return error;
}

static int may_open(struct path *path, int acc_mode, int flag)
{
	struct dentry *dentry = path->dentry;
	struct inode *inode = dentry->d_inode;
	int error;

	
	if (!acc_mode)
		return 0;

	if (!inode)
		return -ENOENT;

	switch (inode->i_mode & S_IFMT) {
	case S_IFLNK:
		return -ELOOP;
	case S_IFDIR:
		if (acc_mode & MAY_WRITE)
			return -EISDIR;
		break;
	case S_IFBLK:
	case S_IFCHR:
		if (path->mnt->mnt_flags & MNT_NODEV)
			return -EACCES;
		
	case S_IFIFO:
	case S_IFSOCK:
		flag &= ~O_TRUNC;
		break;
	}

	error = inode_permission(inode, acc_mode);
	if (error)
		return error;

	if (IS_APPEND(inode)) {
		if  ((flag & O_ACCMODE) != O_RDONLY && !(flag & O_APPEND))
			return -EPERM;
		if (flag & O_TRUNC)
			return -EPERM;
	}

	
	if (flag & O_NOATIME && !inode_owner_or_capable(inode))
		return -EPERM;

	return 0;
}

static int handle_truncate(struct file *filp)
{
	struct path *path = &filp->f_path;
	struct inode *inode = path->dentry->d_inode;
	int error = get_write_access(inode);
	if (error)
		return error;
	error = locks_verify_locked(inode);
	if (!error)
		error = security_path_truncate(path);
	if (!error) {
		error = do_truncate(path->dentry, 0,
				    ATTR_MTIME|ATTR_CTIME|ATTR_OPEN,
				    filp);
	}
	put_write_access(inode);
	return error;
}

static inline int open_to_namei_flags(int flag)
{
	if ((flag & O_ACCMODE) == 3)
		flag--;
	return flag;
}

static struct file *do_last(struct nameidata *nd, struct path *path,
			    const struct open_flags *op, const char *pathname)
{
	struct dentry *dir = nd->path.dentry;
	struct dentry *dentry;
	int open_flag = op->open_flag;
	int will_truncate = open_flag & O_TRUNC;
	int want_write = 0;
	int acc_mode = op->acc_mode;
	struct file *filp;
	int error;

	nd->flags &= ~LOOKUP_PARENT;
	nd->flags |= op->intent;

	switch (nd->last_type) {
	case LAST_DOTDOT:
	case LAST_DOT:
		error = handle_dots(nd, nd->last_type);
		if (error)
			return ERR_PTR(error);
		
	case LAST_ROOT:
		error = complete_walk(nd);
		if (error)
			return ERR_PTR(error);
		audit_inode(pathname, nd->path.dentry);
		if (open_flag & O_CREAT) {
			error = -EISDIR;
			goto exit;
		}
		goto ok;
	case LAST_BIND:
		error = complete_walk(nd);
		if (error)
			return ERR_PTR(error);
		audit_inode(pathname, dir);
		goto ok;
	}

	if (!(open_flag & O_CREAT)) {
		int symlink_ok = 0;
		if (nd->last.name[nd->last.len])
			nd->flags |= LOOKUP_FOLLOW | LOOKUP_DIRECTORY;
		if (open_flag & O_PATH && !(nd->flags & LOOKUP_FOLLOW))
			symlink_ok = 1;
		
		error = walk_component(nd, path, &nd->last, LAST_NORM,
					!symlink_ok);
		if (error < 0)
			return ERR_PTR(error);
		if (error) 
			return NULL;
		
		error = complete_walk(nd);
		if (error)
			return ERR_PTR(error);

		error = -ENOTDIR;
		if (nd->flags & LOOKUP_DIRECTORY) {
			if (!nd->inode->i_op->lookup)
				goto exit;
		}
		audit_inode(pathname, nd->path.dentry);
		goto ok;
	}

	
	error = complete_walk(nd);
	if (error)
		return ERR_PTR(error);

	audit_inode(pathname, dir);
	error = -EISDIR;
	
	if (nd->last.name[nd->last.len])
		goto exit;

	mutex_lock(&dir->d_inode->i_mutex);

	dentry = lookup_hash(nd);
	error = PTR_ERR(dentry);
	if (IS_ERR(dentry)) {
		mutex_unlock(&dir->d_inode->i_mutex);
		goto exit;
	}

	path->dentry = dentry;
	path->mnt = nd->path.mnt;

	
	if (!dentry->d_inode) {
		umode_t mode = op->mode;
		if (!IS_POSIXACL(dir->d_inode))
			mode &= ~current_umask();
		error = mnt_want_write(nd->path.mnt);
		if (error)
			goto exit_mutex_unlock;
		want_write = 1;
		
		open_flag &= ~O_TRUNC;
		will_truncate = 0;
		acc_mode = MAY_OPEN;
		error = security_path_mknod(&nd->path, dentry, mode, 0);
		if (error)
			goto exit_mutex_unlock;
		error = vfs_create(dir->d_inode, dentry, mode, nd);
		if (error)
			goto exit_mutex_unlock;
		mutex_unlock(&dir->d_inode->i_mutex);
		dput(nd->path.dentry);
		nd->path.dentry = dentry;
		goto common;
	}

	mutex_unlock(&dir->d_inode->i_mutex);
	audit_inode(pathname, path->dentry);

	error = -EEXIST;
	if (open_flag & O_EXCL)
		goto exit_dput;

	error = follow_managed(path, nd->flags);
	if (error < 0)
		goto exit_dput;

	if (error)
		nd->flags |= LOOKUP_JUMPED;

	error = -ENOENT;
	if (!path->dentry->d_inode)
		goto exit_dput;

	if (path->dentry->d_inode->i_op->follow_link)
		return NULL;

	path_to_nameidata(path, nd);
	nd->inode = path->dentry->d_inode;
	
	error = complete_walk(nd);
	if (error)
		return ERR_PTR(error);
	error = -EISDIR;
	if (S_ISDIR(nd->inode->i_mode))
		goto exit;
ok:
	if (!S_ISREG(nd->inode->i_mode))
		will_truncate = 0;

	if (will_truncate) {
		error = mnt_want_write(nd->path.mnt);
		if (error)
			goto exit;
		want_write = 1;
	}
common:
	error = may_open(&nd->path, acc_mode, open_flag);
	if (error)
		goto exit;
	filp = nameidata_to_filp(nd);
	if (!IS_ERR(filp)) {
		error = ima_file_check(filp, op->acc_mode);
		if (error) {
			fput(filp);
			filp = ERR_PTR(error);
		}
	}
	if (!IS_ERR(filp)) {
		if (will_truncate) {
			error = handle_truncate(filp);
			if (error) {
				fput(filp);
				filp = ERR_PTR(error);
			}
		}
	}
out:
	if (want_write)
		mnt_drop_write(nd->path.mnt);
	path_put(&nd->path);
	return filp;

exit_mutex_unlock:
	mutex_unlock(&dir->d_inode->i_mutex);
exit_dput:
	path_put_conditional(path, nd);
exit:
	filp = ERR_PTR(error);
	goto out;
}

static struct file *path_openat(int dfd, const char *pathname,
		struct nameidata *nd, const struct open_flags *op, int flags)
{
	struct file *base = NULL;
	struct file *filp;
	struct path path;
	int error;

	filp = get_empty_filp();
	if (!filp)
		return ERR_PTR(-ENFILE);

	filp->f_flags = op->open_flag;
	nd->intent.open.file = filp;
	nd->intent.open.flags = open_to_namei_flags(op->open_flag);
	nd->intent.open.create_mode = op->mode;

	error = path_init(dfd, pathname, flags | LOOKUP_PARENT, nd, &base);
	if (unlikely(error))
		goto out_filp;

	current->total_link_count = 0;
	error = link_path_walk(pathname, nd);
	if (unlikely(error))
		goto out_filp;

	filp = do_last(nd, &path, op, pathname);
	while (unlikely(!filp)) { 
		struct path link = path;
		void *cookie;
		if (!(nd->flags & LOOKUP_FOLLOW)) {
			path_put_conditional(&path, nd);
			path_put(&nd->path);
			filp = ERR_PTR(-ELOOP);
			break;
		}
		nd->flags |= LOOKUP_PARENT;
		nd->flags &= ~(LOOKUP_OPEN|LOOKUP_CREATE|LOOKUP_EXCL);
		error = follow_link(&link, nd, &cookie);
		if (unlikely(error))
			filp = ERR_PTR(error);
		else
			filp = do_last(nd, &path, op, pathname);
		put_link(nd, &link, cookie);
	}
out:
	if (nd->root.mnt && !(nd->flags & LOOKUP_ROOT))
		path_put(&nd->root);
	if (base)
		fput(base);
	release_open_intent(nd);
	return filp;

out_filp:
	filp = ERR_PTR(error);
	goto out;
}

struct file *do_filp_open(int dfd, const char *pathname,
		const struct open_flags *op, int flags)
{
	struct nameidata nd;
	struct file *filp;

	filp = path_openat(dfd, pathname, &nd, op, flags | LOOKUP_RCU);
	if (unlikely(filp == ERR_PTR(-ECHILD)))
		filp = path_openat(dfd, pathname, &nd, op, flags);
	if (unlikely(filp == ERR_PTR(-ESTALE)))
		filp = path_openat(dfd, pathname, &nd, op, flags | LOOKUP_REVAL);
	return filp;
}

struct file *do_file_open_root(struct dentry *dentry, struct vfsmount *mnt,
		const char *name, const struct open_flags *op, int flags)
{
	struct nameidata nd;
	struct file *file;

	nd.root.mnt = mnt;
	nd.root.dentry = dentry;

	flags |= LOOKUP_ROOT;

	if (dentry->d_inode->i_op->follow_link && op->intent & LOOKUP_OPEN)
		return ERR_PTR(-ELOOP);

	file = path_openat(-1, name, &nd, op, flags | LOOKUP_RCU);
	if (unlikely(file == ERR_PTR(-ECHILD)))
		file = path_openat(-1, name, &nd, op, flags);
	if (unlikely(file == ERR_PTR(-ESTALE)))
		file = path_openat(-1, name, &nd, op, flags | LOOKUP_REVAL);
	return file;
}

struct dentry *kern_path_create(int dfd, const char *pathname, struct path *path, int is_dir)
{
	struct dentry *dentry = ERR_PTR(-EEXIST);
	struct nameidata nd;
	int error = do_path_lookup(dfd, pathname, LOOKUP_PARENT, &nd);
	if (error)
		return ERR_PTR(error);

	if (nd.last_type != LAST_NORM)
		goto out;
	nd.flags &= ~LOOKUP_PARENT;
	nd.flags |= LOOKUP_CREATE | LOOKUP_EXCL;
	nd.intent.open.flags = O_EXCL;

	mutex_lock_nested(&nd.path.dentry->d_inode->i_mutex, I_MUTEX_PARENT);
	dentry = lookup_hash(&nd);
	if (IS_ERR(dentry))
		goto fail;

	if (dentry->d_inode)
		goto eexist;
	if (unlikely(!is_dir && nd.last.name[nd.last.len])) {
		dput(dentry);
		dentry = ERR_PTR(-ENOENT);
		goto fail;
	}
	*path = nd.path;
	return dentry;
eexist:
	dput(dentry);
	dentry = ERR_PTR(-EEXIST);
fail:
	mutex_unlock(&nd.path.dentry->d_inode->i_mutex);
out:
	path_put(&nd.path);
	return dentry;
}
EXPORT_SYMBOL(kern_path_create);

struct dentry *user_path_create(int dfd, const char __user *pathname, struct path *path, int is_dir)
{
	char *tmp = getname(pathname);
	struct dentry *res;
	if (IS_ERR(tmp))
		return ERR_CAST(tmp);
	res = kern_path_create(dfd, tmp, path, is_dir);
	putname(tmp);
	return res;
}
EXPORT_SYMBOL(user_path_create);

int vfs_mknod(struct inode *dir, struct dentry *dentry, umode_t mode, dev_t dev)
{
	int error = may_create(dir, dentry);

	if (error)
		return error;

	if ((S_ISCHR(mode) || S_ISBLK(mode)) &&
	    !ns_capable(inode_userns(dir), CAP_MKNOD))
		return -EPERM;

	if (!dir->i_op->mknod)
		return -EPERM;

	error = devcgroup_inode_mknod(mode, dev);
	if (error)
		return error;

	error = security_inode_mknod(dir, dentry, mode, dev);
	if (error)
		return error;

	error = dir->i_op->mknod(dir, dentry, mode, dev);
	if (error)
		return error;

	error = security_inode_post_create(dir, dentry, mode);
	if (error)
		return error;

	if (!error)
		fsnotify_create(dir, dentry);
	return error;
}

static int may_mknod(umode_t mode)
{
	switch (mode & S_IFMT) {
	case S_IFREG:
	case S_IFCHR:
	case S_IFBLK:
	case S_IFIFO:
	case S_IFSOCK:
	case 0: 
		return 0;
	case S_IFDIR:
		return -EPERM;
	default:
		return -EINVAL;
	}
}

SYSCALL_DEFINE4(mknodat, int, dfd, const char __user *, filename, umode_t, mode,
		unsigned, dev)
{
	struct dentry *dentry;
	struct path path;
	int error;

	if (S_ISDIR(mode))
		return -EPERM;

	dentry = user_path_create(dfd, filename, &path, 0);
	if (IS_ERR(dentry))
		return PTR_ERR(dentry);

	if (!IS_POSIXACL(path.dentry->d_inode))
		mode &= ~current_umask();
	error = may_mknod(mode);
	if (error)
		goto out_dput;
	error = mnt_want_write(path.mnt);
	if (error)
		goto out_dput;
	error = security_path_mknod(&path, dentry, mode, dev);
	if (error)
		goto out_drop_write;
	switch (mode & S_IFMT) {
		case 0: case S_IFREG:
			error = vfs_create(path.dentry->d_inode,dentry,mode,NULL);
			break;
		case S_IFCHR: case S_IFBLK:
			error = vfs_mknod(path.dentry->d_inode,dentry,mode,
					new_decode_dev(dev));
			break;
		case S_IFIFO: case S_IFSOCK:
			error = vfs_mknod(path.dentry->d_inode,dentry,mode,0);
			break;
	}
out_drop_write:
	mnt_drop_write(path.mnt);
out_dput:
	dput(dentry);
	mutex_unlock(&path.dentry->d_inode->i_mutex);
	path_put(&path);

	return error;
}

SYSCALL_DEFINE3(mknod, const char __user *, filename, umode_t, mode, unsigned, dev)
{
	return sys_mknodat(AT_FDCWD, filename, mode, dev);
}

int vfs_mkdir(struct inode *dir, struct dentry *dentry, umode_t mode)
{
	int error = may_create(dir, dentry);
	unsigned max_links = dir->i_sb->s_max_links;

	if (error)
		return error;

	if (!dir->i_op->mkdir)
		return -EPERM;

	mode &= (S_IRWXUGO|S_ISVTX);
	error = security_inode_mkdir(dir, dentry, mode);
	if (error)
		return error;

	if (max_links && dir->i_nlink >= max_links)
		return -EMLINK;

	error = dir->i_op->mkdir(dir, dentry, mode);
	if (!error)
		fsnotify_mkdir(dir, dentry);
	return error;
}

SYSCALL_DEFINE3(mkdirat, int, dfd, const char __user *, pathname, umode_t, mode)
{
	struct dentry *dentry;
	struct path path;
	int error;

	dentry = user_path_create(dfd, pathname, &path, 1);
	if (IS_ERR(dentry))
		return PTR_ERR(dentry);

	if (!IS_POSIXACL(path.dentry->d_inode))
		mode &= ~current_umask();
	error = mnt_want_write(path.mnt);
	if (error)
		goto out_dput;
	error = security_path_mkdir(&path, dentry, mode);
	if (error)
		goto out_drop_write;
	error = vfs_mkdir(path.dentry->d_inode, dentry, mode);
out_drop_write:
	mnt_drop_write(path.mnt);
out_dput:
	dput(dentry);
	mutex_unlock(&path.dentry->d_inode->i_mutex);
	path_put(&path);
	return error;
}

SYSCALL_DEFINE2(mkdir, const char __user *, pathname, umode_t, mode)
{
	return sys_mkdirat(AT_FDCWD, pathname, mode);
}

void dentry_unhash(struct dentry *dentry)
{
	shrink_dcache_parent(dentry);
	spin_lock(&dentry->d_lock);
	if (dentry->d_count == 1)
		__d_drop(dentry);
	spin_unlock(&dentry->d_lock);
}

int vfs_rmdir(struct inode *dir, struct dentry *dentry)
{
	int error = may_delete(dir, dentry, 1);

	if (error)
		return error;

	if (!dir->i_op->rmdir)
		return -EPERM;

	dget(dentry);
	mutex_lock(&dentry->d_inode->i_mutex);

	error = -EBUSY;
	if (d_mountpoint(dentry))
		goto out;

	error = security_inode_rmdir(dir, dentry);
	if (error)
		goto out;

	shrink_dcache_parent(dentry);
	error = dir->i_op->rmdir(dir, dentry);
	if (error)
		goto out;

	dentry->d_inode->i_flags |= S_DEAD;
	dont_mount(dentry);

out:
	mutex_unlock(&dentry->d_inode->i_mutex);
	dput(dentry);
	if (!error)
		d_delete(dentry);
	return error;
}

static long do_rmdir(int dfd, const char __user *pathname)
{
	int error = 0;
	char * name;
	struct dentry *dentry;
	struct nameidata nd;

	error = user_path_parent(dfd, pathname, &nd, &name);
	if (error)
		return error;

	switch(nd.last_type) {
	case LAST_DOTDOT:
		error = -ENOTEMPTY;
		goto exit1;
	case LAST_DOT:
		error = -EINVAL;
		goto exit1;
	case LAST_ROOT:
		error = -EBUSY;
		goto exit1;
	}

	nd.flags &= ~LOOKUP_PARENT;

	mutex_lock_nested(&nd.path.dentry->d_inode->i_mutex, I_MUTEX_PARENT);
	dentry = lookup_hash(&nd);
	error = PTR_ERR(dentry);
	if (IS_ERR(dentry))
		goto exit2;
	if (!dentry->d_inode) {
		error = -ENOENT;
		goto exit3;
	}
	error = mnt_want_write(nd.path.mnt);
	if (error)
		goto exit3;
	error = security_path_rmdir(&nd.path, dentry);
	if (error)
		goto exit4;
	error = vfs_rmdir(nd.path.dentry->d_inode, dentry);
exit4:
	mnt_drop_write(nd.path.mnt);
exit3:
	dput(dentry);
exit2:
	mutex_unlock(&nd.path.dentry->d_inode->i_mutex);
exit1:
	path_put(&nd.path);
	putname(name);
	return error;
}

SYSCALL_DEFINE1(rmdir, const char __user *, pathname)
{
	return do_rmdir(AT_FDCWD, pathname);
}

int vfs_unlink(struct inode *dir, struct dentry *dentry)
{
	int error = may_delete(dir, dentry, 0);
	struct super_block *sb = dentry->d_sb;

	if (error)
		return error;

	if (!dir->i_op->unlink)
		return -EPERM;

	trace_vfs_unlink(dentry, dentry->d_inode->i_size);
	mutex_lock(&dentry->d_inode->i_mutex);
	if (d_mountpoint(dentry))
		error = -EBUSY;
	else {
		error = security_inode_unlink(dir, dentry);
		if (!error) {
			error = dir->i_op->unlink(dir, dentry);
			if (!error)
				dont_mount(dentry);
		}
	}
	mutex_unlock(&dentry->d_inode->i_mutex);
	trace_vfs_unlink_done(dentry);
	if (sb && (!strcmp(sb->s_type->name, "ext4")
		|| !strcmp(sb->s_type->name, "fuse")
		|| !strcmp(sb->s_type->name, "vfat")))
		fs_debug_dump(FS_DBG_TYPE_ERASE, dentry->d_inode->i_size);

	
	if (!error && !(dentry->d_flags & DCACHE_NFSFS_RENAMED)) {
		fsnotify_link_count(dentry->d_inode);
		d_delete(dentry);
	}

	return error;
}

static long do_unlinkat(int dfd, const char __user *pathname)
{
	int error;
	char *name;
	struct dentry *dentry;
	struct nameidata nd;
	struct inode *inode = NULL;

	error = user_path_parent(dfd, pathname, &nd, &name);
	if (error)
		return error;

	error = -EISDIR;
	if (nd.last_type != LAST_NORM)
		goto exit1;

	nd.flags &= ~LOOKUP_PARENT;

	mutex_lock_nested(&nd.path.dentry->d_inode->i_mutex, I_MUTEX_PARENT);
	dentry = lookup_hash(&nd);
	error = PTR_ERR(dentry);
	if (!IS_ERR(dentry)) {
		
		if (nd.last.name[nd.last.len])
			goto slashes;
		inode = dentry->d_inode;
		if (!inode)
			goto slashes;
		ihold(inode);
		error = mnt_want_write(nd.path.mnt);
		if (error)
			goto exit2;
		error = security_path_unlink(&nd.path, dentry);
		if (error)
			goto exit3;
		error = vfs_unlink(nd.path.dentry->d_inode, dentry);
exit3:
		mnt_drop_write(nd.path.mnt);
	exit2:
		dput(dentry);
	}
	mutex_unlock(&nd.path.dentry->d_inode->i_mutex);
	if (inode)
		iput(inode);	
exit1:
	path_put(&nd.path);
	putname(name);
	return error;

slashes:
	error = !dentry->d_inode ? -ENOENT :
		S_ISDIR(dentry->d_inode->i_mode) ? -EISDIR : -ENOTDIR;
	goto exit2;
}

SYSCALL_DEFINE3(unlinkat, int, dfd, const char __user *, pathname, int, flag)
{
	if ((flag & ~AT_REMOVEDIR) != 0)
		return -EINVAL;

	if (flag & AT_REMOVEDIR)
		return do_rmdir(dfd, pathname);

	return do_unlinkat(dfd, pathname);
}

SYSCALL_DEFINE1(unlink, const char __user *, pathname)
{
	return do_unlinkat(AT_FDCWD, pathname);
}

int vfs_symlink(struct inode *dir, struct dentry *dentry, const char *oldname)
{
	int error = may_create(dir, dentry);

	if (error)
		return error;

	if (!dir->i_op->symlink)
		return -EPERM;

	error = security_inode_symlink(dir, dentry, oldname);
	if (error)
		return error;

	error = dir->i_op->symlink(dir, dentry, oldname);
	if (!error)
		fsnotify_create(dir, dentry);
	return error;
}

SYSCALL_DEFINE3(symlinkat, const char __user *, oldname,
		int, newdfd, const char __user *, newname)
{
	int error;
	char *from;
	struct dentry *dentry;
	struct path path;

	from = getname(oldname);
	if (IS_ERR(from))
		return PTR_ERR(from);

	dentry = user_path_create(newdfd, newname, &path, 0);
	error = PTR_ERR(dentry);
	if (IS_ERR(dentry))
		goto out_putname;

	error = mnt_want_write(path.mnt);
	if (error)
		goto out_dput;
	error = security_path_symlink(&path, dentry, from);
	if (error)
		goto out_drop_write;
	error = vfs_symlink(path.dentry->d_inode, dentry, from);
out_drop_write:
	mnt_drop_write(path.mnt);
out_dput:
	dput(dentry);
	mutex_unlock(&path.dentry->d_inode->i_mutex);
	path_put(&path);
out_putname:
	putname(from);
	return error;
}

SYSCALL_DEFINE2(symlink, const char __user *, oldname, const char __user *, newname)
{
	return sys_symlinkat(oldname, AT_FDCWD, newname);
}

int vfs_link(struct dentry *old_dentry, struct inode *dir, struct dentry *new_dentry)
{
	struct inode *inode = old_dentry->d_inode;
	unsigned max_links = dir->i_sb->s_max_links;
	int error;

	if (!inode)
		return -ENOENT;

	error = may_create(dir, new_dentry);
	if (error)
		return error;

	if (dir->i_sb != inode->i_sb)
		return -EXDEV;

	if (IS_APPEND(inode) || IS_IMMUTABLE(inode))
		return -EPERM;
	if (!dir->i_op->link)
		return -EPERM;
	if (S_ISDIR(inode->i_mode))
		return -EPERM;

	error = security_inode_link(old_dentry, dir, new_dentry);
	if (error)
		return error;

	mutex_lock(&inode->i_mutex);
	
	if (inode->i_nlink == 0)
		error =  -ENOENT;
	else if (max_links && inode->i_nlink >= max_links)
		error = -EMLINK;
	else
		error = dir->i_op->link(old_dentry, dir, new_dentry);
	mutex_unlock(&inode->i_mutex);
	if (!error)
		fsnotify_link(dir, inode, new_dentry);
	return error;
}

SYSCALL_DEFINE5(linkat, int, olddfd, const char __user *, oldname,
		int, newdfd, const char __user *, newname, int, flags)
{
	struct dentry *new_dentry;
	struct path old_path, new_path;
	int how = 0;
	int error;

	if ((flags & ~(AT_SYMLINK_FOLLOW | AT_EMPTY_PATH)) != 0)
		return -EINVAL;
	if (flags & AT_EMPTY_PATH) {
		if (!capable(CAP_DAC_READ_SEARCH))
			return -ENOENT;
		how = LOOKUP_EMPTY;
	}

	if (flags & AT_SYMLINK_FOLLOW)
		how |= LOOKUP_FOLLOW;

	error = user_path_at(olddfd, oldname, how, &old_path);
	if (error)
		return error;

	new_dentry = user_path_create(newdfd, newname, &new_path, 0);
	error = PTR_ERR(new_dentry);
	if (IS_ERR(new_dentry))
		goto out;

	error = -EXDEV;
	if (old_path.mnt != new_path.mnt)
		goto out_dput;
	error = mnt_want_write(new_path.mnt);
	if (error)
		goto out_dput;
	error = security_path_link(old_path.dentry, &new_path, new_dentry);
	if (error)
		goto out_drop_write;
	error = vfs_link(old_path.dentry, new_path.dentry->d_inode, new_dentry);
out_drop_write:
	mnt_drop_write(new_path.mnt);
out_dput:
	dput(new_dentry);
	mutex_unlock(&new_path.dentry->d_inode->i_mutex);
	path_put(&new_path);
out:
	path_put(&old_path);

	return error;
}

SYSCALL_DEFINE2(link, const char __user *, oldname, const char __user *, newname)
{
	return sys_linkat(AT_FDCWD, oldname, AT_FDCWD, newname, 0);
}

static int vfs_rename_dir(struct inode *old_dir, struct dentry *old_dentry,
			  struct inode *new_dir, struct dentry *new_dentry)
{
	int error = 0;
	struct inode *target = new_dentry->d_inode;
	unsigned max_links = new_dir->i_sb->s_max_links;

	if (new_dir != old_dir) {
		error = inode_permission(old_dentry->d_inode, MAY_WRITE);
		if (error)
			return error;
	}

	error = security_inode_rename(old_dir, old_dentry, new_dir, new_dentry);
	if (error)
		return error;

	dget(new_dentry);
	if (target)
		mutex_lock(&target->i_mutex);

	error = -EBUSY;
	if (d_mountpoint(old_dentry) || d_mountpoint(new_dentry))
		goto out;

	error = -EMLINK;
	if (max_links && !target && new_dir != old_dir &&
	    new_dir->i_nlink >= max_links)
		goto out;

	if (target)
		shrink_dcache_parent(new_dentry);
	error = old_dir->i_op->rename(old_dir, old_dentry, new_dir, new_dentry);
	if (error)
		goto out;

	if (target) {
		target->i_flags |= S_DEAD;
		dont_mount(new_dentry);
	}
out:
	if (target)
		mutex_unlock(&target->i_mutex);
	dput(new_dentry);
	if (!error)
		if (!(old_dir->i_sb->s_type->fs_flags & FS_RENAME_DOES_D_MOVE))
			d_move(old_dentry,new_dentry);
	return error;
}

static int vfs_rename_other(struct inode *old_dir, struct dentry *old_dentry,
			    struct inode *new_dir, struct dentry *new_dentry)
{
	struct inode *target = new_dentry->d_inode;
	int error;

	error = security_inode_rename(old_dir, old_dentry, new_dir, new_dentry);
	if (error)
		return error;

	dget(new_dentry);
	if (target)
		mutex_lock(&target->i_mutex);

	error = -EBUSY;
	if (d_mountpoint(old_dentry)||d_mountpoint(new_dentry))
		goto out;

	error = old_dir->i_op->rename(old_dir, old_dentry, new_dir, new_dentry);
	if (error)
		goto out;

	if (target)
		dont_mount(new_dentry);
	if (!(old_dir->i_sb->s_type->fs_flags & FS_RENAME_DOES_D_MOVE))
		d_move(old_dentry, new_dentry);
out:
	if (target)
		mutex_unlock(&target->i_mutex);
	dput(new_dentry);
	return error;
}

int vfs_rename(struct inode *old_dir, struct dentry *old_dentry,
	       struct inode *new_dir, struct dentry *new_dentry)
{
	int error;
	int is_dir = S_ISDIR(old_dentry->d_inode->i_mode);
	const unsigned char *old_name;

	if (old_dentry->d_inode == new_dentry->d_inode)
 		return 0;
 
	error = may_delete(old_dir, old_dentry, is_dir);
	if (error)
		return error;

	if (!new_dentry->d_inode)
		error = may_create(new_dir, new_dentry);
	else
		error = may_delete(new_dir, new_dentry, is_dir);
	if (error)
		return error;

	if (!old_dir->i_op->rename)
		return -EPERM;

	old_name = fsnotify_oldname_init(old_dentry->d_name.name);

	if (is_dir)
		error = vfs_rename_dir(old_dir,old_dentry,new_dir,new_dentry);
	else
		error = vfs_rename_other(old_dir,old_dentry,new_dir,new_dentry);
	if (!error)
		fsnotify_move(old_dir, new_dir, old_name, is_dir,
			      new_dentry->d_inode, old_dentry);
	fsnotify_oldname_free(old_name);

	return error;
}

SYSCALL_DEFINE4(renameat, int, olddfd, const char __user *, oldname,
		int, newdfd, const char __user *, newname)
{
	struct dentry *old_dir, *new_dir;
	struct dentry *old_dentry, *new_dentry;
	struct dentry *trap;
	struct nameidata oldnd, newnd;
	char *from;
	char *to;
	int error;

	error = user_path_parent(olddfd, oldname, &oldnd, &from);
	if (error)
		goto exit;

	error = user_path_parent(newdfd, newname, &newnd, &to);
	if (error)
		goto exit1;

	error = -EXDEV;
	if (oldnd.path.mnt != newnd.path.mnt)
		goto exit2;

	old_dir = oldnd.path.dentry;
	error = -EBUSY;
	if (oldnd.last_type != LAST_NORM)
		goto exit2;

	new_dir = newnd.path.dentry;
	if (newnd.last_type != LAST_NORM)
		goto exit2;

	oldnd.flags &= ~LOOKUP_PARENT;
	newnd.flags &= ~LOOKUP_PARENT;
	newnd.flags |= LOOKUP_RENAME_TARGET;

	trap = lock_rename(new_dir, old_dir);

	old_dentry = lookup_hash(&oldnd);
	error = PTR_ERR(old_dentry);
	if (IS_ERR(old_dentry))
		goto exit3;
	
	error = -ENOENT;
	if (!old_dentry->d_inode)
		goto exit4;
	
	if (!S_ISDIR(old_dentry->d_inode->i_mode)) {
		error = -ENOTDIR;
		if (oldnd.last.name[oldnd.last.len])
			goto exit4;
		if (newnd.last.name[newnd.last.len])
			goto exit4;
	}
	
	error = -EINVAL;
	if (old_dentry == trap)
		goto exit4;
	new_dentry = lookup_hash(&newnd);
	error = PTR_ERR(new_dentry);
	if (IS_ERR(new_dentry))
		goto exit4;
	
	error = -ENOTEMPTY;
	if (new_dentry == trap)
		goto exit5;

	error = mnt_want_write(oldnd.path.mnt);
	if (error)
		goto exit5;
	error = security_path_rename(&oldnd.path, old_dentry,
				     &newnd.path, new_dentry);
	if (error)
		goto exit6;
	error = vfs_rename(old_dir->d_inode, old_dentry,
				   new_dir->d_inode, new_dentry);
exit6:
	mnt_drop_write(oldnd.path.mnt);
exit5:
	dput(new_dentry);
exit4:
	dput(old_dentry);
exit3:
	unlock_rename(new_dir, old_dir);
exit2:
	path_put(&newnd.path);
	putname(to);
exit1:
	path_put(&oldnd.path);
	putname(from);
exit:
	return error;
}

SYSCALL_DEFINE2(rename, const char __user *, oldname, const char __user *, newname)
{
	return sys_renameat(AT_FDCWD, oldname, AT_FDCWD, newname);
}

int vfs_readlink(struct dentry *dentry, char __user *buffer, int buflen, const char *link)
{
	int len;

	len = PTR_ERR(link);
	if (IS_ERR(link))
		goto out;

	len = strlen(link);
	if (len > (unsigned) buflen)
		len = buflen;
	if (copy_to_user(buffer, link, len))
		len = -EFAULT;
out:
	return len;
}

int generic_readlink(struct dentry *dentry, char __user *buffer, int buflen)
{
	struct nameidata nd;
	void *cookie;
	int res;

	nd.depth = 0;
	cookie = dentry->d_inode->i_op->follow_link(dentry, &nd);
	if (IS_ERR(cookie))
		return PTR_ERR(cookie);

	res = vfs_readlink(dentry, buffer, buflen, nd_get_link(&nd));
	if (dentry->d_inode->i_op->put_link)
		dentry->d_inode->i_op->put_link(dentry, &nd, cookie);
	return res;
}

int vfs_follow_link(struct nameidata *nd, const char *link)
{
	return __vfs_follow_link(nd, link);
}

static char *page_getlink(struct dentry * dentry, struct page **ppage)
{
	char *kaddr;
	struct page *page;
	struct address_space *mapping = dentry->d_inode->i_mapping;
	page = read_mapping_page(mapping, 0, NULL);
	if (IS_ERR(page))
		return (char*)page;
	*ppage = page;
	kaddr = kmap(page);
	nd_terminate_link(kaddr, dentry->d_inode->i_size, PAGE_SIZE - 1);
	return kaddr;
}

int page_readlink(struct dentry *dentry, char __user *buffer, int buflen)
{
	struct page *page = NULL;
	char *s = page_getlink(dentry, &page);
	int res = vfs_readlink(dentry,buffer,buflen,s);
	if (page) {
		kunmap(page);
		page_cache_release(page);
	}
	return res;
}

void *page_follow_link_light(struct dentry *dentry, struct nameidata *nd)
{
	struct page *page = NULL;
	nd_set_link(nd, page_getlink(dentry, &page));
	return page;
}

void page_put_link(struct dentry *dentry, struct nameidata *nd, void *cookie)
{
	struct page *page = cookie;

	if (page) {
		kunmap(page);
		page_cache_release(page);
	}
}

int __page_symlink(struct inode *inode, const char *symname, int len, int nofs)
{
	struct address_space *mapping = inode->i_mapping;
	struct page *page;
	void *fsdata;
	int err;
	char *kaddr;
	unsigned int flags = AOP_FLAG_UNINTERRUPTIBLE;
	if (nofs)
		flags |= AOP_FLAG_NOFS;

retry:
	err = pagecache_write_begin(NULL, mapping, 0, len-1,
				flags, &page, &fsdata);
	if (err)
		goto fail;

	kaddr = kmap_atomic(page);
	memcpy(kaddr, symname, len-1);
	kunmap_atomic(kaddr);

	err = pagecache_write_end(NULL, mapping, 0, len-1, len-1,
							page, fsdata);
	if (err < 0)
		goto fail;
	if (err < len-1)
		goto retry;

	mark_inode_dirty(inode);
	return 0;
fail:
	return err;
}

int page_symlink(struct inode *inode, const char *symname, int len)
{
	return __page_symlink(inode, symname, len,
			!(mapping_gfp_mask(inode->i_mapping) & __GFP_FS));
}

const struct inode_operations page_symlink_inode_operations = {
	.readlink	= generic_readlink,
	.follow_link	= page_follow_link_light,
	.put_link	= page_put_link,
};

EXPORT_SYMBOL(user_path_at);
EXPORT_SYMBOL(follow_down_one);
EXPORT_SYMBOL(follow_down);
EXPORT_SYMBOL(follow_up);
EXPORT_SYMBOL(get_write_access); 
EXPORT_SYMBOL(getname);
EXPORT_SYMBOL(lock_rename);
EXPORT_SYMBOL(lookup_one_len);
EXPORT_SYMBOL(page_follow_link_light);
EXPORT_SYMBOL(page_put_link);
EXPORT_SYMBOL(page_readlink);
EXPORT_SYMBOL(__page_symlink);
EXPORT_SYMBOL(page_symlink);
EXPORT_SYMBOL(page_symlink_inode_operations);
EXPORT_SYMBOL(kern_path);
EXPORT_SYMBOL(vfs_path_lookup);
EXPORT_SYMBOL(inode_permission);
EXPORT_SYMBOL(unlock_rename);
EXPORT_SYMBOL(vfs_create);
EXPORT_SYMBOL(vfs_follow_link);
EXPORT_SYMBOL(vfs_link);
EXPORT_SYMBOL(vfs_mkdir);
EXPORT_SYMBOL(vfs_mknod);
EXPORT_SYMBOL(generic_permission);
EXPORT_SYMBOL(vfs_readlink);
EXPORT_SYMBOL(vfs_rename);
EXPORT_SYMBOL(vfs_rmdir);
EXPORT_SYMBOL(vfs_symlink);
EXPORT_SYMBOL(vfs_unlink);
EXPORT_SYMBOL(dentry_unhash);
EXPORT_SYMBOL(generic_readlink);
