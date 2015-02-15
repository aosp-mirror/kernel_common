/*
 * Copyright (c) 2000-2005 Silicon Graphics, Inc.
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it would be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write the Free Software Foundation,
 * Inc.,  51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */
#include "xfs.h"
#include "xfs_fs.h"
#include "xfs_acl.h"
#include "xfs_bit.h"
#include "xfs_log.h"
#include "xfs_inum.h"
#include "xfs_trans.h"
#include "xfs_sb.h"
#include "xfs_ag.h"
#include "xfs_alloc.h"
#include "xfs_quota.h"
#include "xfs_mount.h"
#include "xfs_bmap_btree.h"
#include "xfs_dinode.h"
#include "xfs_inode.h"
#include "xfs_bmap.h"
#include "xfs_rtalloc.h"
#include "xfs_error.h"
#include "xfs_itable.h"
#include "xfs_rw.h"
#include "xfs_attr.h"
#include "xfs_buf_item.h"
#include "xfs_utils.h"
#include "xfs_vnodeops.h"
#include "xfs_inode_item.h"
#include "xfs_trace.h"

#include <linux/capability.h>
#include <linux/xattr.h>
#include <linux/namei.h>
#include <linux/posix_acl.h>
#include <linux/security.h>
#include <linux/fiemap.h>
#include <linux/slab.h>

static int
xfs_initxattrs(
	struct inode		*inode,
	const struct xattr	*xattr_array,
	void			*fs_info)
{
	const struct xattr	*xattr;
	struct xfs_inode	*ip = XFS_I(inode);
	int			error = 0;

	for (xattr = xattr_array; xattr->name != NULL; xattr++) {
		error = xfs_attr_set(ip, xattr->name, xattr->value,
				     xattr->value_len, ATTR_SECURE);
		if (error < 0)
			break;
	}
	return error;
}

/*
 * Hook in SELinux.  This is not quite correct yet, what we really need
 * here (as we do for default ACLs) is a mechanism by which creation of
 * these attrs can be journalled at inode creation time (along with the
 * inode, of course, such that log replay can't cause these to be lost).
 */

STATIC int
xfs_init_security(
	struct inode	*inode,
	struct inode	*dir,
	const struct qstr *qstr)
{
	return security_inode_init_security(inode, dir, qstr,
					    &xfs_initxattrs, NULL);
}

static void
xfs_dentry_to_name(
	struct xfs_name	*namep,
	struct dentry	*dentry)
{
	namep->name = dentry->d_name.name;
	namep->len = dentry->d_name.len;
}

STATIC void
xfs_cleanup_inode(
	struct inode	*dir,
	struct inode	*inode,
	struct dentry	*dentry)
{
	struct xfs_name	teardown;

	/* Oh, the horror.
	 * If we can't add the ACL or we fail in
	 * xfs_init_security we must back out.
	 * ENOSPC can hit here, among other things.
	 */
	xfs_dentry_to_name(&teardown, dentry);

	xfs_remove(XFS_I(dir), &teardown, XFS_I(inode));
	iput(inode);
}

STATIC int
xfs_vn_mknod(
	struct inode	*dir,
	struct dentry	*dentry,
	umode_t		mode,
	dev_t		rdev)
{
	struct inode	*inode;
	struct xfs_inode *ip = NULL;
	struct posix_acl *default_acl = NULL;
	struct xfs_name	name;
	int		error;

	/*
	 * Irix uses Missed'em'V split, but doesn't want to see
	 * the upper 5 bits of (14bit) major.
	 */
	if (S_ISCHR(mode) || S_ISBLK(mode)) {
		if (unlikely(!sysv_valid_dev(rdev) || MAJOR(rdev) & ~0x1ff))
			return -EINVAL;
		rdev = sysv_encode_dev(rdev);
	} else {
		rdev = 0;
	}

	if (IS_POSIXACL(dir)) {
		default_acl = xfs_get_acl(dir, ACL_TYPE_DEFAULT);
		if (IS_ERR(default_acl))
			return PTR_ERR(default_acl);

		if (!default_acl)
			mode &= ~current_umask();
	}

	xfs_dentry_to_name(&name, dentry);
	error = xfs_create(XFS_I(dir), &name, mode, rdev, &ip);
	if (unlikely(error))
		goto out_free_acl;

	inode = VFS_I(ip);

	error = xfs_init_security(inode, dir, &dentry->d_name);
	if (unlikely(error))
		goto out_cleanup_inode;

	if (default_acl) {
		error = -xfs_inherit_acl(inode, default_acl);
		default_acl = NULL;
		if (unlikely(error))
			goto out_cleanup_inode;
	}


	d_instantiate(dentry, inode);
	return -error;

 out_cleanup_inode:
	xfs_cleanup_inode(dir, inode, dentry);
 out_free_acl:
	posix_acl_release(default_acl);
	return -error;
}

STATIC int
xfs_vn_create(
	struct inode	*dir,
	struct dentry	*dentry,
	umode_t		mode,
	struct nameidata *nd)
{
	return xfs_vn_mknod(dir, dentry, mode, 0);
}

STATIC int
xfs_vn_mkdir(
	struct inode	*dir,
	struct dentry	*dentry,
	umode_t		mode)
{
	return xfs_vn_mknod(dir, dentry, mode|S_IFDIR, 0);
}

STATIC struct dentry *
xfs_vn_lookup(
	struct inode	*dir,
	struct dentry	*dentry,
	struct nameidata *nd)
{
	struct xfs_inode *cip;
	struct xfs_name	name;
	int		error;

	if (dentry->d_name.len >= MAXNAMELEN)
		return ERR_PTR(-ENAMETOOLONG);

	xfs_dentry_to_name(&name, dentry);
	error = xfs_lookup(XFS_I(dir), &name, &cip, NULL);
	if (unlikely(error)) {
		if (unlikely(error != ENOENT))
			return ERR_PTR(-error);
		d_add(dentry, NULL);
		return NULL;
	}

	return d_splice_alias(VFS_I(cip), dentry);
}

STATIC struct dentry *
xfs_vn_ci_lookup(
	struct inode	*dir,
	struct dentry	*dentry,
	struct nameidata *nd)
{
	struct xfs_inode *ip;
	struct xfs_name	xname;
	struct xfs_name ci_name;
	struct qstr	dname;
	int		error;

	if (dentry->d_name.len >= MAXNAMELEN)
		return ERR_PTR(-ENAMETOOLONG);

	xfs_dentry_to_name(&xname, dentry);
	error = xfs_lookup(XFS_I(dir), &xname, &ip, &ci_name);
	if (unlikely(error)) {
		if (unlikely(error != ENOENT))
			return ERR_PTR(-error);
		/*
		 * call d_add(dentry, NULL) here when d_drop_negative_children
		 * is called in xfs_vn_mknod (ie. allow negative dentries
		 * with CI filesystems).
		 */
		return NULL;
	}

	/* if exact match, just splice and exit */
	if (!ci_name.name)
		return d_splice_alias(VFS_I(ip), dentry);

	/* else case-insensitive match... */
	dname.name = ci_name.name;
	dname.len = ci_name.len;
	dentry = d_add_ci(dentry, VFS_I(ip), &dname);
	kmem_free(ci_name.name);
	return dentry;
}

STATIC int
xfs_vn_link(
	struct dentry	*old_dentry,
	struct inode	*dir,
	struct dentry	*dentry)
{
	struct inode	*inode = old_dentry->d_inode;
	struct xfs_name	name;
	int		error;

	xfs_dentry_to_name(&name, dentry);

	error = xfs_link(XFS_I(dir), XFS_I(inode), &name);
	if (unlikely(error))
		return -error;

	ihold(inode);
	d_instantiate(dentry, inode);
	return 0;
}

STATIC int
xfs_vn_unlink(
	struct inode	*dir,
	struct dentry	*dentry)
{
	struct xfs_name	name;
	int		error;

	xfs_dentry_to_name(&name, dentry);

	error = -xfs_remove(XFS_I(dir), &name, XFS_I(dentry->d_inode));
	if (error)
		return error;

	/*
	 * With unlink, the VFS makes the dentry "negative": no inode,
	 * but still hashed. This is incompatible with case-insensitive
	 * mode, so invalidate (unhash) the dentry in CI-mode.
	 */
	if (xfs_sb_version_hasasciici(&XFS_M(dir->i_sb)->m_sb))
		d_invalidate(dentry);
	return 0;
}

STATIC int
xfs_vn_symlink(
	struct inode	*dir,
	struct dentry	*dentry,
	const char	*symname)
{
	struct inode	*inode;
	struct xfs_inode *cip = NULL;
	struct xfs_name	name;
	int		error;
	umode_t		mode;

	mode = S_IFLNK |
		(irix_symlink_mode ? 0777 & ~current_umask() : S_IRWXUGO);
	xfs_dentry_to_name(&name, dentry);

	error = xfs_symlink(XFS_I(dir), &name, symname, mode, &cip);
	if (unlikely(error))
		goto out;

	inode = VFS_I(cip);

	error = xfs_init_security(inode, dir, &dentry->d_name);
	if (unlikely(error))
		goto out_cleanup_inode;

	d_instantiate(dentry, inode);
	return 0;

 out_cleanup_inode:
	xfs_cleanup_inode(dir, inode, dentry);
 out:
	return -error;
}

STATIC int
xfs_vn_rename(
	struct inode	*odir,
	struct dentry	*odentry,
	struct inode	*ndir,
	struct dentry	*ndentry)
{
	struct inode	*new_inode = ndentry->d_inode;
	struct xfs_name	oname;
	struct xfs_name	nname;

	xfs_dentry_to_name(&oname, odentry);
	xfs_dentry_to_name(&nname, ndentry);

	return -xfs_rename(XFS_I(odir), &oname, XFS_I(odentry->d_inode),
			   XFS_I(ndir), &nname, new_inode ?
			   			XFS_I(new_inode) : NULL);
}

/*
 * careful here - this function can get called recursively, so
 * we need to be very careful about how much stack we use.
 * uio is kmalloced for this reason...
 */
STATIC void *
xfs_vn_follow_link(
	struct dentry		*dentry,
	struct nameidata	*nd)
{
	char			*link;
	int			error = -ENOMEM;

	link = kmalloc(MAXPATHLEN+1, GFP_KERNEL);
	if (!link)
		goto out_err;

	error = -xfs_readlink(XFS_I(dentry->d_inode), link);
	if (unlikely(error))
		goto out_kfree;

	nd_set_link(nd, link);
	return NULL;

 out_kfree:
	kfree(link);
 out_err:
	nd_set_link(nd, ERR_PTR(error));
	return NULL;
}

STATIC void
xfs_vn_put_link(
	struct dentry	*dentry,
	struct nameidata *nd,
	void		*p)
{
	char		*s = nd_get_link(nd);

	if (!IS_ERR(s))
		kfree(s);
}

STATIC int
xfs_vn_getattr(
	struct vfsmount		*mnt,
	struct dentry		*dentry,
	struct kstat		*stat)
{
	struct inode		*inode = dentry->d_inode;
	struct xfs_inode	*ip = XFS_I(inode);
	struct xfs_mount	*mp = ip->i_mount;

	trace_xfs_getattr(ip);

	if (XFS_FORCED_SHUTDOWN(mp))
		return -XFS_ERROR(EIO);

	stat->size = XFS_ISIZE(ip);
	stat->dev = inode->i_sb->s_dev;
	stat->mode = ip->i_d.di_mode;
	stat->nlink = ip->i_d.di_nlink;
	stat->uid = ip->i_d.di_uid;
	stat->gid = ip->i_d.di_gid;
	stat->ino = ip->i_ino;
	stat->atime = inode->i_atime;
	stat->mtime = inode->i_mtime;
	stat->ctime = inode->i_ctime;
	stat->blocks =
		XFS_FSB_TO_BB(mp, ip->i_d.di_nblocks + ip->i_delayed_blks);


	switch (inode->i_mode & S_IFMT) {
	case S_IFBLK:
	case S_IFCHR:
		stat->blksize = BLKDEV_IOSIZE;
		stat->rdev = MKDEV(sysv_major(ip->i_df.if_u2.if_rdev) & 0x1ff,
				   sysv_minor(ip->i_df.if_u2.if_rdev));
		break;
	default:
		if (XFS_IS_REALTIME_INODE(ip)) {
			/*
			 * If the file blocks are being allocated from a
			 * realtime volume, then return the inode's realtime
			 * extent size or the realtime volume's extent size.
			 */
			stat->blksize =
				xfs_get_extsz_hint(ip) << mp->m_sb.sb_blocklog;
		} else
			stat->blksize = xfs_preferred_iosize(mp);
		stat->rdev = 0;
		break;
	}

	return 0;
}

int
xfs_setattr_nonsize(
	struct xfs_inode	*ip,
	struct iattr		*iattr,
	int			flags)
{
	xfs_mount_t		*mp = ip->i_mount;
	struct inode		*inode = VFS_I(ip);
	int			mask = iattr->ia_valid;
	xfs_trans_t		*tp;
	int			error;
	uid_t			uid = 0, iuid = 0;
	gid_t			gid = 0, igid = 0;
	struct xfs_dquot	*udqp = NULL, *gdqp = NULL;
	struct xfs_dquot	*olddquot1 = NULL, *olddquot2 = NULL;

	trace_xfs_setattr(ip);

	if (mp->m_flags & XFS_MOUNT_RDONLY)
		return XFS_ERROR(EROFS);

	if (XFS_FORCED_SHUTDOWN(mp))
		return XFS_ERROR(EIO);

	error = -inode_change_ok(inode, iattr);
	if (error)
		return XFS_ERROR(error);

	ASSERT((mask & ATTR_SIZE) == 0);

	/*
	 * If disk quotas is on, we make sure that the dquots do exist on disk,
	 * before we start any other transactions. Trying to do this later
	 * is messy. We don't care to take a readlock to look at the ids
	 * in inode here, because we can't hold it across the trans_reserve.
	 * If the IDs do change before we take the ilock, we're covered
	 * because the i_*dquot fields will get updated anyway.
	 */
	if (XFS_IS_QUOTA_ON(mp) && (mask & (ATTR_UID|ATTR_GID))) {
		uint	qflags = 0;

		if ((mask & ATTR_UID) && XFS_IS_UQUOTA_ON(mp)) {
			uid = iattr->ia_uid;
			qflags |= XFS_QMOPT_UQUOTA;
		} else {
			uid = ip->i_d.di_uid;
		}
		if ((mask & ATTR_GID) && XFS_IS_GQUOTA_ON(mp)) {
			gid = iattr->ia_gid;
			qflags |= XFS_QMOPT_GQUOTA;
		}  else {
			gid = ip->i_d.di_gid;
		}

		/*
		 * We take a reference when we initialize udqp and gdqp,
		 * so it is important that we never blindly double trip on
		 * the same variable. See xfs_create() for an example.
		 */
		ASSERT(udqp == NULL);
		ASSERT(gdqp == NULL);
		error = xfs_qm_vop_dqalloc(ip, uid, gid, xfs_get_projid(ip),
					 qflags, &udqp, &gdqp);
		if (error)
			return error;
	}

	tp = xfs_trans_alloc(mp, XFS_TRANS_SETATTR_NOT_SIZE);
	error = xfs_trans_reserve(tp, 0, XFS_ICHANGE_LOG_RES(mp), 0, 0, 0);
	if (error)
		goto out_dqrele;

	xfs_ilock(ip, XFS_ILOCK_EXCL);

	/*
	 * Change file ownership.  Must be the owner or privileged.
	 */
	if (mask & (ATTR_UID|ATTR_GID)) {
		/*
		 * These IDs could have changed since we last looked at them.
		 * But, we're assured that if the ownership did change
		 * while we didn't have the inode locked, inode's dquot(s)
		 * would have changed also.
		 */
		iuid = ip->i_d.di_uid;
		igid = ip->i_d.di_gid;
		gid = (mask & ATTR_GID) ? iattr->ia_gid : igid;
		uid = (mask & ATTR_UID) ? iattr->ia_uid : iuid;

		/*
		 * Do a quota reservation only if uid/gid is actually
		 * going to change.
		 */
		if (XFS_IS_QUOTA_RUNNING(mp) &&
		    ((XFS_IS_UQUOTA_ON(mp) && iuid != uid) ||
		     (XFS_IS_GQUOTA_ON(mp) && igid != gid))) {
			ASSERT(tp);
			error = xfs_qm_vop_chown_reserve(tp, ip, udqp, gdqp,
						capable(CAP_FOWNER) ?
						XFS_QMOPT_FORCE_RES : 0);
			if (error)	/* out of quota */
				goto out_trans_cancel;
		}
	}

	xfs_trans_ijoin(tp, ip, 0);

	/*
	 * Change file ownership.  Must be the owner or privileged.
	 */
	if (mask & (ATTR_UID|ATTR_GID)) {
		/*
		 * CAP_FSETID overrides the following restrictions:
		 *
		 * The set-user-ID and set-group-ID bits of a file will be
		 * cleared upon successful return from chown()
		 */
		if ((ip->i_d.di_mode & (S_ISUID|S_ISGID)) &&
		    !capable(CAP_FSETID))
			ip->i_d.di_mode &= ~(S_ISUID|S_ISGID);

		/*
		 * Change the ownerships and register quota modifications
		 * in the transaction.
		 */
		if (iuid != uid) {
			if (XFS_IS_QUOTA_RUNNING(mp) && XFS_IS_UQUOTA_ON(mp)) {
				ASSERT(mask & ATTR_UID);
				ASSERT(udqp);
				olddquot1 = xfs_qm_vop_chown(tp, ip,
							&ip->i_udquot, udqp);
			}
			ip->i_d.di_uid = uid;
			inode->i_uid = uid;
		}
		if (igid != gid) {
			if (XFS_IS_QUOTA_RUNNING(mp) && XFS_IS_GQUOTA_ON(mp)) {
				ASSERT(!XFS_IS_PQUOTA_ON(mp));
				ASSERT(mask & ATTR_GID);
				ASSERT(gdqp);
				olddquot2 = xfs_qm_vop_chown(tp, ip,
							&ip->i_gdquot, gdqp);
			}
			ip->i_d.di_gid = gid;
			inode->i_gid = gid;
		}
	}

	/*
	 * Change file access modes.
	 */
	if (mask & ATTR_MODE) {
		umode_t mode = iattr->ia_mode;

		if (!in_group_p(inode->i_gid) && !capable(CAP_FSETID))
			mode &= ~S_ISGID;

		ip->i_d.di_mode &= S_IFMT;
		ip->i_d.di_mode |= mode & ~S_IFMT;

		inode->i_mode &= S_IFMT;
		inode->i_mode |= mode & ~S_IFMT;
	}

	/*
	 * Change file access or modified times.
	 */
	if (mask & ATTR_ATIME) {
		inode->i_atime = iattr->ia_atime;
		ip->i_d.di_atime.t_sec = iattr->ia_atime.tv_sec;
		ip->i_d.di_atime.t_nsec = iattr->ia_atime.tv_nsec;
	}
	if (mask & ATTR_CTIME) {
		inode->i_ctime = iattr->ia_ctime;
		ip->i_d.di_ctime.t_sec = iattr->ia_ctime.tv_sec;
		ip->i_d.di_ctime.t_nsec = iattr->ia_ctime.tv_nsec;
	}
	if (mask & ATTR_MTIME) {
		inode->i_mtime = iattr->ia_mtime;
		ip->i_d.di_mtime.t_sec = iattr->ia_mtime.tv_sec;
		ip->i_d.di_mtime.t_nsec = iattr->ia_mtime.tv_nsec;
	}

	xfs_trans_log_inode(tp, ip, XFS_ILOG_CORE);

	XFS_STATS_INC(xs_ig_attrchg);

	if (mp->m_flags & XFS_MOUNT_WSYNC)
		xfs_trans_set_sync(tp);
	error = xfs_trans_commit(tp, 0);

	xfs_iunlock(ip, XFS_ILOCK_EXCL);

	/*
	 * Release any dquot(s) the inode had kept before chown.
	 */
	xfs_qm_dqrele(olddquot1);
	xfs_qm_dqrele(olddquot2);
	xfs_qm_dqrele(udqp);
	xfs_qm_dqrele(gdqp);

	if (error)
		return XFS_ERROR(error);

	/*
	 * XXX(hch): Updating the ACL entries is not atomic vs the i_mode
	 * 	     update.  We could avoid this with linked transactions
	 * 	     and passing down the transaction pointer all the way
	 *	     to attr_set.  No previous user of the generic
	 * 	     Posix ACL code seems to care about this issue either.
	 */
	if ((mask & ATTR_MODE) && !(flags & XFS_ATTR_NOACL)) {
		error = -xfs_acl_chmod(inode);
		if (error)
			return XFS_ERROR(error);
	}

	return 0;

out_trans_cancel:
	xfs_trans_cancel(tp, 0);
	xfs_iunlock(ip, XFS_ILOCK_EXCL);
out_dqrele:
	xfs_qm_dqrele(udqp);
	xfs_qm_dqrele(gdqp);
	return error;
}

/*
 * Truncate file.  Must have write permission and not be a directory.
 */
int
xfs_setattr_size(
	struct xfs_inode	*ip,
	struct iattr		*iattr,
	int			flags)
{
	struct xfs_mount	*mp = ip->i_mount;
	struct inode		*inode = VFS_I(ip);
	int			mask = iattr->ia_valid;
	xfs_off_t		oldsize, newsize;
	struct xfs_trans	*tp;
	int			error;
	uint			lock_flags;
	uint			commit_flags = 0;

	trace_xfs_setattr(ip);

	if (mp->m_flags & XFS_MOUNT_RDONLY)
		return XFS_ERROR(EROFS);

	if (XFS_FORCED_SHUTDOWN(mp))
		return XFS_ERROR(EIO);

	error = -inode_change_ok(inode, iattr);
	if (error)
		return XFS_ERROR(error);

	ASSERT(S_ISREG(ip->i_d.di_mode));
	ASSERT((mask & (ATTR_MODE|ATTR_UID|ATTR_GID|ATTR_ATIME|ATTR_ATIME_SET|
			ATTR_MTIME_SET|ATTR_KILL_SUID|ATTR_KILL_SGID|
			ATTR_KILL_PRIV|ATTR_TIMES_SET)) == 0);

	lock_flags = XFS_ILOCK_EXCL;
	if (!(flags & XFS_ATTR_NOLOCK))
		lock_flags |= XFS_IOLOCK_EXCL;
	xfs_ilock(ip, lock_flags);

	oldsize = inode->i_size;
	newsize = iattr->ia_size;

	/*
	 * Short circuit the truncate case for zero length files.
	 */
	if (newsize == 0 && oldsize == 0 && ip->i_d.di_nextents == 0) {
		if (!(mask & (ATTR_CTIME|ATTR_MTIME)))
			goto out_unlock;

		/*
		 * Use the regular setattr path to update the timestamps.
		 */
		xfs_iunlock(ip, lock_flags);
		iattr->ia_valid &= ~ATTR_SIZE;
		return xfs_setattr_nonsize(ip, iattr, 0);
	}

	/*
	 * Make sure that the dquots are attached to the inode.
	 */
	error = xfs_qm_dqattach_locked(ip, 0);
	if (error)
		goto out_unlock;

	/*
	 * Now we can make the changes.  Before we join the inode to the
	 * transaction, take care of the part of the truncation that must be
	 * done without the inode lock.  This needs to be done before joining
	 * the inode to the transaction, because the inode cannot be unlocked
	 * once it is a part of the transaction.
	 */
	if (newsize > oldsize) {
		/*
		 * Do the first part of growing a file: zero any data in the
		 * last block that is beyond the old EOF.  We need to do this
		 * before the inode is joined to the transaction to modify
		 * i_size.
		 */
		error = xfs_zero_eof(ip, newsize, oldsize);
		if (error)
			goto out_unlock;
	}
	xfs_iunlock(ip, XFS_ILOCK_EXCL);
	lock_flags &= ~XFS_ILOCK_EXCL;

	/*
	 * We are going to log the inode size change in this transaction so
	 * any previous writes that are beyond the on disk EOF and the new
	 * EOF that have not been written out need to be written here.  If we
	 * do not write the data out, we expose ourselves to the null files
	 * problem.
	 *
	 * Only flush from the on disk size to the smaller of the in memory
	 * file size or the new size as that's the range we really care about
	 * here and prevents waiting for other data not within the range we
	 * care about here.
	 */
	if (oldsize != ip->i_d.di_size && newsize > ip->i_d.di_size) {
		error = xfs_flush_pages(ip, ip->i_d.di_size, newsize, 0,
					FI_NONE);
		if (error)
			goto out_unlock;
	}

	/*
	 * Wait for all direct I/O to complete.
	 */
	inode_dio_wait(inode);

	error = -block_truncate_page(inode->i_mapping, newsize, xfs_get_blocks);
	if (error)
		goto out_unlock;

	tp = xfs_trans_alloc(mp, XFS_TRANS_SETATTR_SIZE);
	error = xfs_trans_reserve(tp, 0, XFS_ITRUNCATE_LOG_RES(mp), 0,
				 XFS_TRANS_PERM_LOG_RES,
				 XFS_ITRUNCATE_LOG_COUNT);
	if (error)
		goto out_trans_cancel;

	truncate_setsize(inode, newsize);

	commit_flags = XFS_TRANS_RELEASE_LOG_RES;
	lock_flags |= XFS_ILOCK_EXCL;

	xfs_ilock(ip, XFS_ILOCK_EXCL);

	xfs_trans_ijoin(tp, ip, 0);

	/*
	 * Only change the c/mtime if we are changing the size or we are
	 * explicitly asked to change it.  This handles the semantic difference
	 * between truncate() and ftruncate() as implemented in the VFS.
	 *
	 * The regular truncate() case without ATTR_CTIME and ATTR_MTIME is a
	 * special case where we need to update the times despite not having
	 * these flags set.  For all other operations the VFS set these flags
	 * explicitly if it wants a timestamp update.
	 */
	if (newsize != oldsize && (!(mask & (ATTR_CTIME | ATTR_MTIME)))) {
		iattr->ia_ctime = iattr->ia_mtime =
			current_fs_time(inode->i_sb);
		mask |= ATTR_CTIME | ATTR_MTIME;
	}

	/*
	 * The first thing we do is set the size to new_size permanently on
	 * disk.  This way we don't have to worry about anyone ever being able
	 * to look at the data being freed even in the face of a crash.
	 * What we're getting around here is the case where we free a block, it
	 * is allocated to another file, it is written to, and then we crash.
	 * If the new data gets written to the file but the log buffers
	 * containing the free and reallocation don't, then we'd end up with
	 * garbage in the blocks being freed.  As long as we make the new size
	 * permanent before actually freeing any blocks it doesn't matter if
	 * they get written to.
	 */
	ip->i_d.di_size = newsize;
	xfs_trans_log_inode(tp, ip, XFS_ILOG_CORE);

	if (newsize <= oldsize) {
		error = xfs_itruncate_extents(&tp, ip, XFS_DATA_FORK, newsize);
		if (error)
			goto out_trans_abort;

		/*
		 * Truncated "down", so we're removing references to old data
		 * here - if we delay flushing for a long time, we expose
		 * ourselves unduly to the notorious NULL files problem.  So,
		 * we mark this inode and flush it when the file is closed,
		 * and do not wait the usual (long) time for writeout.
		 */
		xfs_iflags_set(ip, XFS_ITRUNCATED);
	}

	if (mask & ATTR_CTIME) {
		inode->i_ctime = iattr->ia_ctime;
		ip->i_d.di_ctime.t_sec = iattr->ia_ctime.tv_sec;
		ip->i_d.di_ctime.t_nsec = iattr->ia_ctime.tv_nsec;
	}
	if (mask & ATTR_MTIME) {
		inode->i_mtime = iattr->ia_mtime;
		ip->i_d.di_mtime.t_sec = iattr->ia_mtime.tv_sec;
		ip->i_d.di_mtime.t_nsec = iattr->ia_mtime.tv_nsec;
	}

	xfs_trans_log_inode(tp, ip, XFS_ILOG_CORE);

	XFS_STATS_INC(xs_ig_attrchg);

	if (mp->m_flags & XFS_MOUNT_WSYNC)
		xfs_trans_set_sync(tp);

	error = xfs_trans_commit(tp, XFS_TRANS_RELEASE_LOG_RES);
out_unlock:
	if (lock_flags)
		xfs_iunlock(ip, lock_flags);
	return error;

out_trans_abort:
	commit_flags |= XFS_TRANS_ABORT;
out_trans_cancel:
	xfs_trans_cancel(tp, commit_flags);
	goto out_unlock;
}

STATIC int
xfs_vn_setattr(
	struct dentry	*dentry,
	struct iattr	*iattr)
{
	if (iattr->ia_valid & ATTR_SIZE)
		return -xfs_setattr_size(XFS_I(dentry->d_inode), iattr, 0);
	return -xfs_setattr_nonsize(XFS_I(dentry->d_inode), iattr, 0);
}

#define XFS_FIEMAP_FLAGS	(FIEMAP_FLAG_SYNC|FIEMAP_FLAG_XATTR)

/*
 * Call fiemap helper to fill in user data.
 * Returns positive errors to xfs_getbmap.
 */
STATIC int
xfs_fiemap_format(
	void			**arg,
	struct getbmapx		*bmv,
	int			*full)
{
	int			error;
	struct fiemap_extent_info *fieinfo = *arg;
	u32			fiemap_flags = 0;
	u64			logical, physical, length;

	/* Do nothing for a hole */
	if (bmv->bmv_block == -1LL)
		return 0;

	logical = BBTOB(bmv->bmv_offset);
	physical = BBTOB(bmv->bmv_block);
	length = BBTOB(bmv->bmv_length);

	if (bmv->bmv_oflags & BMV_OF_PREALLOC)
		fiemap_flags |= FIEMAP_EXTENT_UNWRITTEN;
	else if (bmv->bmv_oflags & BMV_OF_DELALLOC) {
		fiemap_flags |= FIEMAP_EXTENT_DELALLOC;
		physical = 0;   /* no block yet */
	}
	if (bmv->bmv_oflags & BMV_OF_LAST)
		fiemap_flags |= FIEMAP_EXTENT_LAST;

	error = fiemap_fill_next_extent(fieinfo, logical, physical,
					length, fiemap_flags);
	if (error > 0) {
		error = 0;
		*full = 1;	/* user array now full */
	}

	return -error;
}

STATIC int
xfs_vn_fiemap(
	struct inode		*inode,
	struct fiemap_extent_info *fieinfo,
	u64			start,
	u64			length)
{
	xfs_inode_t		*ip = XFS_I(inode);
	struct getbmapx		bm;
	int			error;

	error = fiemap_check_flags(fieinfo, XFS_FIEMAP_FLAGS);
	if (error)
		return error;

	/* Set up bmap header for xfs internal routine */
	bm.bmv_offset = BTOBB(start);
	/* Special case for whole file */
	if (length == FIEMAP_MAX_OFFSET)
		bm.bmv_length = -1LL;
	else
		bm.bmv_length = BTOBB(length);

	/* We add one because in getbmap world count includes the header */
	bm.bmv_count = !fieinfo->fi_extents_max ? MAXEXTNUM :
					fieinfo->fi_extents_max + 1;
	bm.bmv_count = min_t(__s32, bm.bmv_count,
			     (PAGE_SIZE * 16 / sizeof(struct getbmapx)));
	bm.bmv_iflags = BMV_IF_PREALLOC | BMV_IF_NO_HOLES;
	if (fieinfo->fi_flags & FIEMAP_FLAG_XATTR)
		bm.bmv_iflags |= BMV_IF_ATTRFORK;
	if (!(fieinfo->fi_flags & FIEMAP_FLAG_SYNC))
		bm.bmv_iflags |= BMV_IF_DELALLOC;

	error = xfs_getbmap(ip, &bm, xfs_fiemap_format, fieinfo);
	if (error)
		return -error;

	return 0;
}

static const struct inode_operations xfs_inode_operations = {
	.get_acl		= xfs_get_acl,
	.getattr		= xfs_vn_getattr,
	.setattr		= xfs_vn_setattr,
	.setxattr		= generic_setxattr,
	.getxattr		= generic_getxattr,
	.removexattr		= generic_removexattr,
	.listxattr		= xfs_vn_listxattr,
	.fiemap			= xfs_vn_fiemap,
};

static const struct inode_operations xfs_dir_inode_operations = {
	.create			= xfs_vn_create,
	.lookup			= xfs_vn_lookup,
	.link			= xfs_vn_link,
	.unlink			= xfs_vn_unlink,
	.symlink		= xfs_vn_symlink,
	.mkdir			= xfs_vn_mkdir,
	/*
	 * Yes, XFS uses the same method for rmdir and unlink.
	 *
	 * There are some subtile differences deeper in the code,
	 * but we use S_ISDIR to check for those.
	 */
	.rmdir			= xfs_vn_unlink,
	.mknod			= xfs_vn_mknod,
	.rename			= xfs_vn_rename,
	.get_acl		= xfs_get_acl,
	.getattr		= xfs_vn_getattr,
	.setattr		= xfs_vn_setattr,
	.setxattr		= generic_setxattr,
	.getxattr		= generic_getxattr,
	.removexattr		= generic_removexattr,
	.listxattr		= xfs_vn_listxattr,
};

static const struct inode_operations xfs_dir_ci_inode_operations = {
	.create			= xfs_vn_create,
	.lookup			= xfs_vn_ci_lookup,
	.link			= xfs_vn_link,
	.unlink			= xfs_vn_unlink,
	.symlink		= xfs_vn_symlink,
	.mkdir			= xfs_vn_mkdir,
	/*
	 * Yes, XFS uses the same method for rmdir and unlink.
	 *
	 * There are some subtile differences deeper in the code,
	 * but we use S_ISDIR to check for those.
	 */
	.rmdir			= xfs_vn_unlink,
	.mknod			= xfs_vn_mknod,
	.rename			= xfs_vn_rename,
	.get_acl		= xfs_get_acl,
	.getattr		= xfs_vn_getattr,
	.setattr		= xfs_vn_setattr,
	.setxattr		= generic_setxattr,
	.getxattr		= generic_getxattr,
	.removexattr		= generic_removexattr,
	.listxattr		= xfs_vn_listxattr,
};

static const struct inode_operations xfs_symlink_inode_operations = {
	.readlink		= generic_readlink,
	.follow_link		= xfs_vn_follow_link,
	.put_link		= xfs_vn_put_link,
	.get_acl		= xfs_get_acl,
	.getattr		= xfs_vn_getattr,
	.setattr		= xfs_vn_setattr,
	.setxattr		= generic_setxattr,
	.getxattr		= generic_getxattr,
	.removexattr		= generic_removexattr,
	.listxattr		= xfs_vn_listxattr,
};

STATIC void
xfs_diflags_to_iflags(
	struct inode		*inode,
	struct xfs_inode	*ip)
{
	if (ip->i_d.di_flags & XFS_DIFLAG_IMMUTABLE)
		inode->i_flags |= S_IMMUTABLE;
	else
		inode->i_flags &= ~S_IMMUTABLE;
	if (ip->i_d.di_flags & XFS_DIFLAG_APPEND)
		inode->i_flags |= S_APPEND;
	else
		inode->i_flags &= ~S_APPEND;
	if (ip->i_d.di_flags & XFS_DIFLAG_SYNC)
		inode->i_flags |= S_SYNC;
	else
		inode->i_flags &= ~S_SYNC;
	if (ip->i_d.di_flags & XFS_DIFLAG_NOATIME)
		inode->i_flags |= S_NOATIME;
	else
		inode->i_flags &= ~S_NOATIME;
}

/*
 * Initialize the Linux inode, set up the operation vectors and
 * unlock the inode.
 *
 * When reading existing inodes from disk this is called directly
 * from xfs_iget, when creating a new inode it is called from
 * xfs_ialloc after setting up the inode.
 *
 * We are always called with an uninitialised linux inode here.
 * We need to initialise the necessary fields and take a reference
 * on it.
 */
void
xfs_setup_inode(
	struct xfs_inode	*ip)
{
	struct inode		*inode = &ip->i_vnode;

	inode->i_ino = ip->i_ino;
	inode->i_state = I_NEW;

	inode_sb_list_add(inode);
	/* make the inode look hashed for the writeback code */
	hlist_add_fake(&inode->i_hash);

	inode->i_mode	= ip->i_d.di_mode;
	set_nlink(inode, ip->i_d.di_nlink);
	inode->i_uid	= ip->i_d.di_uid;
	inode->i_gid	= ip->i_d.di_gid;

	switch (inode->i_mode & S_IFMT) {
	case S_IFBLK:
	case S_IFCHR:
		inode->i_rdev =
			MKDEV(sysv_major(ip->i_df.if_u2.if_rdev) & 0x1ff,
			      sysv_minor(ip->i_df.if_u2.if_rdev));
		break;
	default:
		inode->i_rdev = 0;
		break;
	}

	inode->i_generation = ip->i_d.di_gen;
	i_size_write(inode, ip->i_d.di_size);
	inode->i_atime.tv_sec	= ip->i_d.di_atime.t_sec;
	inode->i_atime.tv_nsec	= ip->i_d.di_atime.t_nsec;
	inode->i_mtime.tv_sec	= ip->i_d.di_mtime.t_sec;
	inode->i_mtime.tv_nsec	= ip->i_d.di_mtime.t_nsec;
	inode->i_ctime.tv_sec	= ip->i_d.di_ctime.t_sec;
	inode->i_ctime.tv_nsec	= ip->i_d.di_ctime.t_nsec;
	xfs_diflags_to_iflags(inode, ip);

	switch (inode->i_mode & S_IFMT) {
	case S_IFREG:
		inode->i_op = &xfs_inode_operations;
		inode->i_fop = &xfs_file_operations;
		inode->i_mapping->a_ops = &xfs_address_space_operations;
		break;
	case S_IFDIR:
		if (xfs_sb_version_hasasciici(&XFS_M(inode->i_sb)->m_sb))
			inode->i_op = &xfs_dir_ci_inode_operations;
		else
			inode->i_op = &xfs_dir_inode_operations;
		inode->i_fop = &xfs_dir_file_operations;
		break;
	case S_IFLNK:
		inode->i_op = &xfs_symlink_inode_operations;
		if (!(ip->i_df.if_flags & XFS_IFINLINE))
			inode->i_mapping->a_ops = &xfs_address_space_operations;
		break;
	default:
		inode->i_op = &xfs_inode_operations;
		init_special_inode(inode, inode->i_mode, inode->i_rdev);
		break;
	}

	/*
	 * If there is no attribute fork no ACL can exist on this inode,
	 * and it can't have any file capabilities attached to it either.
	 */
	if (!XFS_IFORK_Q(ip)) {
		inode_has_no_xattr(inode);
		cache_no_acl(inode);
	}

	xfs_iflags_clear(ip, XFS_INEW);
	barrier();

	unlock_new_inode(inode);
}
