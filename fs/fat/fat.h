#ifndef _FAT_H
#define _FAT_H

#include <linux/buffer_head.h>
#include <linux/string.h>
#include <linux/nls.h>
#include <linux/fs.h>
#include <linux/mutex.h>
#include <linux/ratelimit.h>
#include <linux/msdos_fs.h>

#define VFAT_SFN_DISPLAY_LOWER	0x0001 
#define VFAT_SFN_DISPLAY_WIN95	0x0002 
#define VFAT_SFN_DISPLAY_WINNT	0x0004 
#define VFAT_SFN_CREATE_WIN95	0x0100 
#define VFAT_SFN_CREATE_WINNT	0x0200 

#define FAT_ERRORS_CONT		1      
#define FAT_ERRORS_PANIC	2      
#define FAT_ERRORS_RO		3      

#define FAT_CHARSET_ERROR	9999

#define FAT_IOCTL_MOVE_CLUSTERS	_IOW('r', 0x16, __u32)

struct fat_mount_options {
	uid_t fs_uid;
	gid_t fs_gid;
	unsigned short fs_fmask;
	unsigned short fs_dmask;
	unsigned short codepage;  
	char *iocharset;          
	unsigned short shortname; 
	unsigned char name_check; 
	unsigned char errors;	  
	unsigned short allow_utime;
	unsigned quiet:1,         
		 showexec:1,      
		 sys_immutable:1, 
		 dotsOK:1,        
		 isvfat:1,        
		 utf8:1,	  
		 unicode_xlate:1, 
		 numtail:1,       
		 flush:1,	  
		 nocase:1,	  
		 usefree:1,	  
		 tz_utc:1,	  
		 rodir:1,	  
		 discard:1;	  
};

#define FAT_HASH_BITS	8
#define FAT_HASH_SIZE	(1UL << FAT_HASH_BITS)

struct msdos_sb_info {
	unsigned short sec_per_clus; 
	unsigned short cluster_bits; 
	unsigned int cluster_size;   
	unsigned char fats,fat_bits; 
	unsigned short fat_start;
	unsigned long fat_length;    
	unsigned long dir_start;
	unsigned short dir_entries;  
	unsigned long data_start;    
	unsigned long max_cluster;   
	unsigned long root_cluster;  
	unsigned long fsinfo_sector; 
	struct mutex fat_lock;
	unsigned int prev_free;      
	unsigned int free_clusters;  
	unsigned int free_clus_valid; 
	struct fat_mount_options options;
	struct nls_table *nls_disk;  
	struct nls_table *nls_io;    
	const void *dir_ops;		     
	int dir_per_block;	     
	int dir_per_block_bits;	     
	unsigned long vol_id;        

	int fatent_shift;
	struct fatent_operations *fatent_ops;
	struct inode *fat_inode;

	struct ratelimit_state ratelimit;

	spinlock_t inode_hash_lock;
	struct hlist_head inode_hashtable[FAT_HASH_SIZE];
};

#define FAT_CACHE_VALID	0	

struct msdos_inode_info {
	spinlock_t cache_lru_lock;
	struct list_head cache_lru;
	int nr_caches;
	
	unsigned int cache_valid_id;

	
	loff_t mmu_private;	

	int i_start;		
	int i_logstart;		
	int i_attrs;		
	loff_t i_pos;		
	struct hlist_node i_fat_hash;	
	struct rw_semaphore truncate_lock; 
	struct inode vfs_inode;
};

struct fat_slot_info {
	loff_t i_pos;		
	loff_t slot_off;	
	int nr_slots;		
	struct msdos_dir_entry *de;
	struct buffer_head *bh;
};

static inline struct msdos_sb_info *MSDOS_SB(struct super_block *sb)
{
	return sb->s_fs_info;
}

static inline struct msdos_inode_info *MSDOS_I(struct inode *inode)
{
	return container_of(inode, struct msdos_inode_info, vfs_inode);
}

static inline int fat_mode_can_hold_ro(struct inode *inode)
{
	struct msdos_sb_info *sbi = MSDOS_SB(inode->i_sb);
	umode_t mask;

	if (S_ISDIR(inode->i_mode)) {
		if (!sbi->options.rodir)
			return 0;
		mask = ~sbi->options.fs_dmask;
	} else
		mask = ~sbi->options.fs_fmask;

	if (!(mask & S_IWUGO))
		return 0;
	return 1;
}

static inline umode_t fat_make_mode(struct msdos_sb_info *sbi,
				   u8 attrs, umode_t mode)
{
	if (attrs & ATTR_RO && !((attrs & ATTR_DIR) && !sbi->options.rodir))
		mode &= ~S_IWUGO;

	if (attrs & ATTR_DIR)
		return (mode & ~sbi->options.fs_dmask) | S_IFDIR;
	else
		return (mode & ~sbi->options.fs_fmask) | S_IFREG;
}

static inline u8 fat_make_attrs(struct inode *inode)
{
	u8 attrs = MSDOS_I(inode)->i_attrs;
	if (S_ISDIR(inode->i_mode))
		attrs |= ATTR_DIR;
	if (fat_mode_can_hold_ro(inode) && !(inode->i_mode & S_IWUGO))
		attrs |= ATTR_RO;
	return attrs;
}

static inline void fat_save_attrs(struct inode *inode, u8 attrs)
{
	if (fat_mode_can_hold_ro(inode))
		MSDOS_I(inode)->i_attrs = attrs & ATTR_UNUSED;
	else
		MSDOS_I(inode)->i_attrs = attrs & (ATTR_UNUSED | ATTR_RO);
}

static inline unsigned char fat_checksum(const __u8 *name)
{
	unsigned char s = name[0];
	s = (s<<7) + (s>>1) + name[1];	s = (s<<7) + (s>>1) + name[2];
	s = (s<<7) + (s>>1) + name[3];	s = (s<<7) + (s>>1) + name[4];
	s = (s<<7) + (s>>1) + name[5];	s = (s<<7) + (s>>1) + name[6];
	s = (s<<7) + (s>>1) + name[7];	s = (s<<7) + (s>>1) + name[8];
	s = (s<<7) + (s>>1) + name[9];	s = (s<<7) + (s>>1) + name[10];
	return s;
}

static inline sector_t fat_clus_to_blknr(struct msdos_sb_info *sbi, int clus)
{
	return ((sector_t)clus - FAT_START_ENT) * sbi->sec_per_clus
		+ sbi->data_start;
}

static inline void fat16_towchar(wchar_t *dst, const __u8 *src, size_t len)
{
#ifdef __BIG_ENDIAN
	while (len--) {
		*dst++ = src[0] | (src[1] << 8);
		src += 2;
	}
#else
	memcpy(dst, src, len * 2);
#endif
}

static inline void fatwchar_to16(__u8 *dst, const wchar_t *src, size_t len)
{
#ifdef __BIG_ENDIAN
	while (len--) {
		dst[0] = *src & 0x00FF;
		dst[1] = (*src & 0xFF00) >> 8;
		dst += 2;
		src++;
	}
#else
	memcpy(dst, src, len * 2);
#endif
}

extern void fat_cache_inval_inode(struct inode *inode);
extern int fat_get_cluster(struct inode *inode, int cluster,
			   int *fclus, int *dclus);
extern int fat_bmap(struct inode *inode, sector_t sector, sector_t *phys,
		    unsigned long *mapped_blocks, int create);

extern const struct file_operations fat_dir_operations;
extern int fat_search_long(struct inode *inode, const unsigned char *name,
			   int name_len, struct fat_slot_info *sinfo);
extern int fat_dir_empty(struct inode *dir);
extern int fat_subdirs(struct inode *dir);
extern int fat_scan(struct inode *dir, const unsigned char *name,
		    struct fat_slot_info *sinfo);
extern int fat_get_dotdot_entry(struct inode *dir, struct buffer_head **bh,
				struct msdos_dir_entry **de, loff_t *i_pos);
extern int fat_alloc_new_dir(struct inode *dir, struct timespec *ts);
extern int fat_add_entries(struct inode *dir, void *slots, int nr_slots,
			   struct fat_slot_info *sinfo);
extern int fat_remove_entries(struct inode *dir, struct fat_slot_info *sinfo);

struct fat_entry {
	int entry;
	union {
		u8 *ent12_p[2];
		__le16 *ent16_p;
		__le32 *ent32_p;
	} u;
	int nr_bhs;
	struct buffer_head *bhs[2];
	struct inode *fat_inode;
};

static inline void fatent_init(struct fat_entry *fatent)
{
	fatent->nr_bhs = 0;
	fatent->entry = 0;
	fatent->u.ent32_p = NULL;
	fatent->bhs[0] = fatent->bhs[1] = NULL;
	fatent->fat_inode = NULL;
}

static inline void fatent_set_entry(struct fat_entry *fatent, int entry)
{
	fatent->entry = entry;
	fatent->u.ent32_p = NULL;
}

static inline void fatent_brelse(struct fat_entry *fatent)
{
	int i;
	fatent->u.ent32_p = NULL;
	for (i = 0; i < fatent->nr_bhs; i++)
		brelse(fatent->bhs[i]);
	fatent->nr_bhs = 0;
	fatent->bhs[0] = fatent->bhs[1] = NULL;
	fatent->fat_inode = NULL;
}

extern void fat_ent_access_init(struct super_block *sb);
extern int fat_ent_read(struct inode *inode, struct fat_entry *fatent,
			int entry);
extern int fat_ent_write(struct inode *inode, struct fat_entry *fatent,
			 int new, int wait);
extern int fat_alloc_clusters(struct inode *inode, int *cluster,
			      int nr_cluster);
extern int fat_free_clusters(struct inode *inode, int cluster);
extern int fat_count_free_clusters(struct super_block *sb);

extern long fat_generic_ioctl(struct file *filp, unsigned int cmd,
			      unsigned long arg);
extern const struct file_operations fat_file_operations;
extern const struct inode_operations fat_file_inode_operations;
extern int fat_setattr(struct dentry * dentry, struct iattr * attr);
extern void fat_truncate_blocks(struct inode *inode, loff_t offset);
extern int fat_getattr(struct vfsmount *mnt, struct dentry *dentry,
		       struct kstat *stat);
extern int fat_file_fsync(struct file *file, loff_t start, loff_t end,
			  int datasync);

extern void fat_attach(struct inode *inode, loff_t i_pos);
extern void fat_detach(struct inode *inode);
extern struct inode *fat_iget(struct super_block *sb, loff_t i_pos);
extern struct inode *fat_build_inode(struct super_block *sb,
			struct msdos_dir_entry *de, loff_t i_pos);
extern int fat_sync_inode(struct inode *inode);
extern int fat_fill_super(struct super_block *sb, void *data, int silent,
			  int isvfat, void (*setup)(struct super_block *));

extern int fat_flush_inodes(struct super_block *sb, struct inode *i1,
		            struct inode *i2);
extern __printf(3, 4) __cold
void __fat_fs_error(struct super_block *sb, int report, const char *fmt, ...);
#define fat_fs_error_ratelimit(sb, fmt, args...) \
	__fat_fs_error(sb, __ratelimit(&MSDOS_SB(sb)->ratelimit), fmt , ## args)
#define fat_fs_error(sb, fmt, args...)	fat_fs_error_ratelimit(sb, fmt, ## args)
__printf(3, 4) __cold
void fat_msg(struct super_block *sb, const char *level, const char *fmt, ...);
extern int fat_clusters_flush(struct super_block *sb);
extern int fat_chain_add(struct inode *inode, int new_dclus, int nr_cluster);
extern void fat_time_fat2unix(struct msdos_sb_info *sbi, struct timespec *ts,
			      __le16 __time, __le16 __date, u8 time_cs);
extern void fat_time_unix2fat(struct msdos_sb_info *sbi, struct timespec *ts,
			      __le16 *time, __le16 *date, u8 *time_cs);
extern int fat_sync_bhs(struct buffer_head **bhs, int nr_bhs);

extern int fat_ioctl_move_cluster(struct file *filp, u32 __user *user_arg);

int fat_cache_init(void);
void fat_cache_destroy(void);

typedef unsigned long long	llu;

#endif 
