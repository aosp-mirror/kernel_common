/*
 * linux/fs/binfmt_elf.c
 *
 * These are the functions used to load ELF format executables as used
 * on SVr4 machines.  Information on the format may be found in the book
 * "UNIX SYSTEM V RELEASE 4 Programmers Guide: Ansi C and Programming Support
 * Tools".
 *
 * Copyright 1993, 1994: Eric Youngdale (ericy@cais.com).
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/errno.h>
#include <linux/signal.h>
#include <linux/binfmts.h>
#include <linux/string.h>
#include <linux/file.h>
#include <linux/slab.h>
#include <linux/personality.h>
#include <linux/elfcore.h>
#include <linux/init.h>
#include <linux/highuid.h>
#include <linux/compiler.h>
#include <linux/highmem.h>
#include <linux/pagemap.h>
#include <linux/security.h>
#include <linux/random.h>
#include <linux/elf.h>
#include <linux/utsname.h>
#include <linux/coredump.h>
#include <asm/uaccess.h>
#include <asm/param.h>
#include <asm/page.h>
#include <asm/exec.h>

static int load_elf_binary(struct linux_binprm *bprm, struct pt_regs *regs);
static int load_elf_library(struct file *);
static unsigned long elf_map(struct file *, unsigned long, struct elf_phdr *,
				int, int, unsigned long);

#ifdef CONFIG_ELF_CORE
static int elf_core_dump(struct coredump_params *cprm);
#else
#define elf_core_dump	NULL
#endif

#if ELF_EXEC_PAGESIZE > PAGE_SIZE
#define ELF_MIN_ALIGN	ELF_EXEC_PAGESIZE
#else
#define ELF_MIN_ALIGN	PAGE_SIZE
#endif

#ifndef ELF_CORE_EFLAGS
#define ELF_CORE_EFLAGS	0
#endif

#define ELF_PAGESTART(_v) ((_v) & ~(unsigned long)(ELF_MIN_ALIGN-1))
#define ELF_PAGEOFFSET(_v) ((_v) & (ELF_MIN_ALIGN-1))
#define ELF_PAGEALIGN(_v) (((_v) + ELF_MIN_ALIGN - 1) & ~(ELF_MIN_ALIGN - 1))

static struct linux_binfmt elf_format = {
	.module		= THIS_MODULE,
	.load_binary	= load_elf_binary,
	.load_shlib	= load_elf_library,
	.core_dump	= elf_core_dump,
	.min_coredump	= ELF_EXEC_PAGESIZE,
};

#define BAD_ADDR(x) ((unsigned long)(x) >= TASK_SIZE)

static int set_brk(unsigned long start, unsigned long end)
{
	start = ELF_PAGEALIGN(start);
	end = ELF_PAGEALIGN(end);
	if (end > start) {
		unsigned long addr;
		addr = vm_brk(start, end - start);
		if (BAD_ADDR(addr))
			return addr;
	}
	current->mm->start_brk = current->mm->brk = end;
	return 0;
}

static int padzero(unsigned long elf_bss)
{
	unsigned long nbyte;

	nbyte = ELF_PAGEOFFSET(elf_bss);
	if (nbyte) {
		nbyte = ELF_MIN_ALIGN - nbyte;
		if (clear_user((void __user *) elf_bss, nbyte))
			return -EFAULT;
	}
	return 0;
}

#ifdef CONFIG_STACK_GROWSUP
#define STACK_ADD(sp, items) ((elf_addr_t __user *)(sp) + (items))
#define STACK_ROUND(sp, items) \
	((15 + (unsigned long) ((sp) + (items))) &~ 15UL)
#define STACK_ALLOC(sp, len) ({ \
	elf_addr_t __user *old_sp = (elf_addr_t __user *)sp; sp += len; \
	old_sp; })
#else
#define STACK_ADD(sp, items) ((elf_addr_t __user *)(sp) - (items))
#define STACK_ROUND(sp, items) \
	(((unsigned long) (sp - items)) &~ 15UL)
#define STACK_ALLOC(sp, len) ({ sp -= len ; sp; })
#endif

#ifndef ELF_BASE_PLATFORM
#define ELF_BASE_PLATFORM NULL
#endif

static int
create_elf_tables(struct linux_binprm *bprm, struct elfhdr *exec,
		unsigned long load_addr, unsigned long interp_load_addr)
{
	unsigned long p = bprm->p;
	int argc = bprm->argc;
	int envc = bprm->envc;
	elf_addr_t __user *argv;
	elf_addr_t __user *envp;
	elf_addr_t __user *sp;
	elf_addr_t __user *u_platform;
	elf_addr_t __user *u_base_platform;
	elf_addr_t __user *u_rand_bytes;
	const char *k_platform = ELF_PLATFORM;
	const char *k_base_platform = ELF_BASE_PLATFORM;
	unsigned char k_rand_bytes[16];
	int items;
	elf_addr_t *elf_info;
	int ei_index = 0;
	const struct cred *cred = current_cred();
	struct vm_area_struct *vma;


	p = arch_align_stack(p);

	u_platform = NULL;
	if (k_platform) {
		size_t len = strlen(k_platform) + 1;

		u_platform = (elf_addr_t __user *)STACK_ALLOC(p, len);
		if (__copy_to_user(u_platform, k_platform, len))
			return -EFAULT;
	}

	u_base_platform = NULL;
	if (k_base_platform) {
		size_t len = strlen(k_base_platform) + 1;

		u_base_platform = (elf_addr_t __user *)STACK_ALLOC(p, len);
		if (__copy_to_user(u_base_platform, k_base_platform, len))
			return -EFAULT;
	}

	get_random_bytes(k_rand_bytes, sizeof(k_rand_bytes));
	u_rand_bytes = (elf_addr_t __user *)
		       STACK_ALLOC(p, sizeof(k_rand_bytes));
	if (__copy_to_user(u_rand_bytes, k_rand_bytes, sizeof(k_rand_bytes)))
		return -EFAULT;

	
	elf_info = (elf_addr_t *)current->mm->saved_auxv;
	
#define NEW_AUX_ENT(id, val) \
	do { \
		elf_info[ei_index++] = id; \
		elf_info[ei_index++] = val; \
	} while (0)

#ifdef ARCH_DLINFO
	ARCH_DLINFO;
#endif
	NEW_AUX_ENT(AT_HWCAP, ELF_HWCAP);
	NEW_AUX_ENT(AT_PAGESZ, ELF_EXEC_PAGESIZE);
	NEW_AUX_ENT(AT_CLKTCK, CLOCKS_PER_SEC);
	NEW_AUX_ENT(AT_PHDR, load_addr + exec->e_phoff);
	NEW_AUX_ENT(AT_PHENT, sizeof(struct elf_phdr));
	NEW_AUX_ENT(AT_PHNUM, exec->e_phnum);
	NEW_AUX_ENT(AT_BASE, interp_load_addr);
	NEW_AUX_ENT(AT_FLAGS, 0);
	NEW_AUX_ENT(AT_ENTRY, exec->e_entry);
	NEW_AUX_ENT(AT_UID, cred->uid);
	NEW_AUX_ENT(AT_EUID, cred->euid);
	NEW_AUX_ENT(AT_GID, cred->gid);
	NEW_AUX_ENT(AT_EGID, cred->egid);
 	NEW_AUX_ENT(AT_SECURE, security_bprm_secureexec(bprm));
	NEW_AUX_ENT(AT_RANDOM, (elf_addr_t)(unsigned long)u_rand_bytes);
	NEW_AUX_ENT(AT_EXECFN, bprm->exec);
	if (k_platform) {
		NEW_AUX_ENT(AT_PLATFORM,
			    (elf_addr_t)(unsigned long)u_platform);
	}
	if (k_base_platform) {
		NEW_AUX_ENT(AT_BASE_PLATFORM,
			    (elf_addr_t)(unsigned long)u_base_platform);
	}
	if (bprm->interp_flags & BINPRM_FLAGS_EXECFD) {
		NEW_AUX_ENT(AT_EXECFD, bprm->interp_data);
	}
#undef NEW_AUX_ENT
	
	memset(&elf_info[ei_index], 0,
	       sizeof current->mm->saved_auxv - ei_index * sizeof elf_info[0]);

	
	ei_index += 2;

	sp = STACK_ADD(p, ei_index);

	items = (argc + 1) + (envc + 1) + 1;
	bprm->p = STACK_ROUND(sp, items);

	
#ifdef CONFIG_STACK_GROWSUP
	sp = (elf_addr_t __user *)bprm->p - items - ei_index;
	bprm->exec = (unsigned long)sp; 
#else
	sp = (elf_addr_t __user *)bprm->p;
#endif


	vma = find_extend_vma(current->mm, bprm->p);
	if (!vma)
		return -EFAULT;

	
	if (__put_user(argc, sp++))
		return -EFAULT;
	argv = sp;
	envp = argv + argc + 1;

	
	p = current->mm->arg_end = current->mm->arg_start;
	while (argc-- > 0) {
		size_t len;
		if (__put_user((elf_addr_t)p, argv++))
			return -EFAULT;
		len = strnlen_user((void __user *)p, MAX_ARG_STRLEN);
		if (!len || len > MAX_ARG_STRLEN)
			return -EINVAL;
		p += len;
	}
	if (__put_user(0, argv))
		return -EFAULT;
	current->mm->arg_end = current->mm->env_start = p;
	while (envc-- > 0) {
		size_t len;
		if (__put_user((elf_addr_t)p, envp++))
			return -EFAULT;
		len = strnlen_user((void __user *)p, MAX_ARG_STRLEN);
		if (!len || len > MAX_ARG_STRLEN)
			return -EINVAL;
		p += len;
	}
	if (__put_user(0, envp))
		return -EFAULT;
	current->mm->env_end = p;

	
	sp = (elf_addr_t __user *)envp + 1;
	if (copy_to_user(sp, elf_info, ei_index * sizeof(elf_addr_t)))
		return -EFAULT;
	return 0;
}

static unsigned long elf_map(struct file *filep, unsigned long addr,
		struct elf_phdr *eppnt, int prot, int type,
		unsigned long total_size)
{
	unsigned long map_addr;
	unsigned long size = eppnt->p_filesz + ELF_PAGEOFFSET(eppnt->p_vaddr);
	unsigned long off = eppnt->p_offset - ELF_PAGEOFFSET(eppnt->p_vaddr);
	addr = ELF_PAGESTART(addr);
	size = ELF_PAGEALIGN(size);

	if (!size)
		return addr;

	down_write(&current->mm->mmap_sem);
	if (total_size) {
		total_size = ELF_PAGEALIGN(total_size);
		map_addr = do_mmap(filep, addr, total_size, prot, type, off);
		if (!BAD_ADDR(map_addr))
			do_munmap(current->mm, map_addr+size, total_size-size);
	} else
		map_addr = do_mmap(filep, addr, size, prot, type, off);

	up_write(&current->mm->mmap_sem);
	return(map_addr);
}

static unsigned long total_mapping_size(struct elf_phdr *cmds, int nr)
{
	int i, first_idx = -1, last_idx = -1;

	for (i = 0; i < nr; i++) {
		if (cmds[i].p_type == PT_LOAD) {
			last_idx = i;
			if (first_idx == -1)
				first_idx = i;
		}
	}
	if (first_idx == -1)
		return 0;

	return cmds[last_idx].p_vaddr + cmds[last_idx].p_memsz -
				ELF_PAGESTART(cmds[first_idx].p_vaddr);
}



static unsigned long load_elf_interp(struct elfhdr *interp_elf_ex,
		struct file *interpreter, unsigned long *interp_map_addr,
		unsigned long no_base)
{
	struct elf_phdr *elf_phdata;
	struct elf_phdr *eppnt;
	unsigned long load_addr = 0;
	int load_addr_set = 0;
	unsigned long last_bss = 0, elf_bss = 0;
	unsigned long error = ~0UL;
	unsigned long total_size;
	int retval, i, size;

	
	if (interp_elf_ex->e_type != ET_EXEC &&
	    interp_elf_ex->e_type != ET_DYN)
		goto out;
	if (!elf_check_arch(interp_elf_ex))
		goto out;
	if (!interpreter->f_op || !interpreter->f_op->mmap)
		goto out;

	if (interp_elf_ex->e_phentsize != sizeof(struct elf_phdr))
		goto out;
	if (interp_elf_ex->e_phnum < 1 ||
		interp_elf_ex->e_phnum > 65536U / sizeof(struct elf_phdr))
		goto out;

	
	size = sizeof(struct elf_phdr) * interp_elf_ex->e_phnum;
	if (size > ELF_MIN_ALIGN)
		goto out;
	elf_phdata = kmalloc(size, GFP_KERNEL);
	if (!elf_phdata)
		goto out;

	retval = kernel_read(interpreter, interp_elf_ex->e_phoff,
			     (char *)elf_phdata, size);
	error = -EIO;
	if (retval != size) {
		if (retval < 0)
			error = retval;	
		goto out_close;
	}

	total_size = total_mapping_size(elf_phdata, interp_elf_ex->e_phnum);
	if (!total_size) {
		error = -EINVAL;
		goto out_close;
	}

	eppnt = elf_phdata;
	for (i = 0; i < interp_elf_ex->e_phnum; i++, eppnt++) {
		if (eppnt->p_type == PT_LOAD) {
			int elf_type = MAP_PRIVATE | MAP_DENYWRITE;
			int elf_prot = 0;
			unsigned long vaddr = 0;
			unsigned long k, map_addr;

			if (eppnt->p_flags & PF_R)
		    		elf_prot = PROT_READ;
			if (eppnt->p_flags & PF_W)
				elf_prot |= PROT_WRITE;
			if (eppnt->p_flags & PF_X)
				elf_prot |= PROT_EXEC;
			vaddr = eppnt->p_vaddr;
			if (interp_elf_ex->e_type == ET_EXEC || load_addr_set)
				elf_type |= MAP_FIXED;
			else if (no_base && interp_elf_ex->e_type == ET_DYN)
				load_addr = -vaddr;

			map_addr = elf_map(interpreter, load_addr + vaddr,
					eppnt, elf_prot, elf_type, total_size);
			total_size = 0;
			if (!*interp_map_addr)
				*interp_map_addr = map_addr;
			error = map_addr;
			if (BAD_ADDR(map_addr))
				goto out_close;

			if (!load_addr_set &&
			    interp_elf_ex->e_type == ET_DYN) {
				load_addr = map_addr - ELF_PAGESTART(vaddr);
				load_addr_set = 1;
			}

			k = load_addr + eppnt->p_vaddr;
			if (BAD_ADDR(k) ||
			    eppnt->p_filesz > eppnt->p_memsz ||
			    eppnt->p_memsz > TASK_SIZE ||
			    TASK_SIZE - eppnt->p_memsz < k) {
				error = -ENOMEM;
				goto out_close;
			}

			k = load_addr + eppnt->p_vaddr + eppnt->p_filesz;
			if (k > elf_bss)
				elf_bss = k;

			k = load_addr + eppnt->p_memsz + eppnt->p_vaddr;
			if (k > last_bss)
				last_bss = k;
		}
	}

	if (last_bss > elf_bss) {
		if (padzero(elf_bss)) {
			error = -EFAULT;
			goto out_close;
		}

		
		elf_bss = ELF_PAGESTART(elf_bss + ELF_MIN_ALIGN - 1);

		
		error = vm_brk(elf_bss, last_bss - elf_bss);
		if (BAD_ADDR(error))
			goto out_close;
	}

	error = load_addr;

out_close:
	kfree(elf_phdata);
out:
	return error;
}


#define INTERPRETER_NONE 0
#define INTERPRETER_ELF 2

#ifndef STACK_RND_MASK
#define STACK_RND_MASK (0x7ff >> (PAGE_SHIFT - 12))	
#endif

static unsigned long randomize_stack_top(unsigned long stack_top)
{
	unsigned int random_variable = 0;

	if ((current->flags & PF_RANDOMIZE) &&
		!(current->personality & ADDR_NO_RANDOMIZE)) {
		random_variable = get_random_int() & STACK_RND_MASK;
		random_variable <<= PAGE_SHIFT;
	}
#ifdef CONFIG_STACK_GROWSUP
	return PAGE_ALIGN(stack_top) + random_variable;
#else
	return PAGE_ALIGN(stack_top) - random_variable;
#endif
}

static int load_elf_binary(struct linux_binprm *bprm, struct pt_regs *regs)
{
	struct file *interpreter = NULL; 
 	unsigned long load_addr = 0, load_bias = 0;
	int load_addr_set = 0;
	char * elf_interpreter = NULL;
	unsigned long error;
	struct elf_phdr *elf_ppnt, *elf_phdata;
	unsigned long elf_bss, elf_brk;
	int retval, i;
	unsigned int size;
	unsigned long elf_entry;
	unsigned long interp_load_addr = 0;
	unsigned long start_code, end_code, start_data, end_data;
	unsigned long reloc_func_desc __maybe_unused = 0;
	int executable_stack = EXSTACK_DEFAULT;
	unsigned long def_flags = 0;
	struct {
		struct elfhdr elf_ex;
		struct elfhdr interp_elf_ex;
	} *loc;

	loc = kmalloc(sizeof(*loc), GFP_KERNEL);
	if (!loc) {
		retval = -ENOMEM;
		goto out_ret;
	}
	
	
	loc->elf_ex = *((struct elfhdr *)bprm->buf);

	retval = -ENOEXEC;
	
	if (memcmp(loc->elf_ex.e_ident, ELFMAG, SELFMAG) != 0)
		goto out;

	if (loc->elf_ex.e_type != ET_EXEC && loc->elf_ex.e_type != ET_DYN)
		goto out;
	if (!elf_check_arch(&loc->elf_ex))
		goto out;
	if (!bprm->file->f_op || !bprm->file->f_op->mmap)
		goto out;

	
	if (loc->elf_ex.e_phentsize != sizeof(struct elf_phdr))
		goto out;
	if (loc->elf_ex.e_phnum < 1 ||
	 	loc->elf_ex.e_phnum > 65536U / sizeof(struct elf_phdr))
		goto out;
	size = loc->elf_ex.e_phnum * sizeof(struct elf_phdr);
	retval = -ENOMEM;
	elf_phdata = kmalloc(size, GFP_KERNEL);
	if (!elf_phdata)
		goto out;

	retval = kernel_read(bprm->file, loc->elf_ex.e_phoff,
			     (char *)elf_phdata, size);
	if (retval != size) {
		if (retval >= 0)
			retval = -EIO;
		goto out_free_ph;
	}

	elf_ppnt = elf_phdata;
	elf_bss = 0;
	elf_brk = 0;

	start_code = ~0UL;
	end_code = 0;
	start_data = 0;
	end_data = 0;

	for (i = 0; i < loc->elf_ex.e_phnum; i++) {
		if (elf_ppnt->p_type == PT_INTERP) {
			retval = -ENOEXEC;
			if (elf_ppnt->p_filesz > PATH_MAX || 
			    elf_ppnt->p_filesz < 2)
				goto out_free_ph;

			retval = -ENOMEM;
			elf_interpreter = kmalloc(elf_ppnt->p_filesz,
						  GFP_KERNEL);
			if (!elf_interpreter)
				goto out_free_ph;

			retval = kernel_read(bprm->file, elf_ppnt->p_offset,
					     elf_interpreter,
					     elf_ppnt->p_filesz);
			if (retval != elf_ppnt->p_filesz) {
				if (retval >= 0)
					retval = -EIO;
				goto out_free_interp;
			}
			
			retval = -ENOEXEC;
			if (elf_interpreter[elf_ppnt->p_filesz - 1] != '\0')
				goto out_free_interp;

			interpreter = open_exec(elf_interpreter);
			retval = PTR_ERR(interpreter);
			if (IS_ERR(interpreter))
				goto out_free_interp;

			would_dump(bprm, interpreter);

			retval = kernel_read(interpreter, 0, bprm->buf,
					     BINPRM_BUF_SIZE);
			if (retval != BINPRM_BUF_SIZE) {
				if (retval >= 0)
					retval = -EIO;
				goto out_free_dentry;
			}

			
			loc->interp_elf_ex = *((struct elfhdr *)bprm->buf);
			break;
		}
		elf_ppnt++;
	}

	elf_ppnt = elf_phdata;
	for (i = 0; i < loc->elf_ex.e_phnum; i++, elf_ppnt++)
		if (elf_ppnt->p_type == PT_GNU_STACK) {
			if (elf_ppnt->p_flags & PF_X)
				executable_stack = EXSTACK_ENABLE_X;
			else
				executable_stack = EXSTACK_DISABLE_X;
			break;
		}

	
	if (elf_interpreter) {
		retval = -ELIBBAD;
		
		if (memcmp(loc->interp_elf_ex.e_ident, ELFMAG, SELFMAG) != 0)
			goto out_free_dentry;
		
		if (!elf_check_arch(&loc->interp_elf_ex))
			goto out_free_dentry;
	}

	
	retval = flush_old_exec(bprm);
	if (retval)
		goto out_free_dentry;

	
	current->mm->def_flags = def_flags;

	SET_PERSONALITY(loc->elf_ex);
	if (elf_read_implies_exec(loc->elf_ex, executable_stack))
		current->personality |= READ_IMPLIES_EXEC;

	if (!(current->personality & ADDR_NO_RANDOMIZE) && randomize_va_space)
		current->flags |= PF_RANDOMIZE;

	setup_new_exec(bprm);

	current->mm->free_area_cache = current->mm->mmap_base;
	current->mm->cached_hole_size = 0;
	retval = setup_arg_pages(bprm, randomize_stack_top(STACK_TOP),
				 executable_stack);
	if (retval < 0) {
		send_sig(SIGKILL, current, 0);
		goto out_free_dentry;
	}
	
	current->mm->start_stack = bprm->p;

	for(i = 0, elf_ppnt = elf_phdata;
	    i < loc->elf_ex.e_phnum; i++, elf_ppnt++) {
		int elf_prot = 0, elf_flags;
		unsigned long k, vaddr;

		if (elf_ppnt->p_type != PT_LOAD)
			continue;

		if (unlikely (elf_brk > elf_bss)) {
			unsigned long nbyte;
	            
			retval = set_brk(elf_bss + load_bias,
					 elf_brk + load_bias);
			if (retval) {
				send_sig(SIGKILL, current, 0);
				goto out_free_dentry;
			}
			nbyte = ELF_PAGEOFFSET(elf_bss);
			if (nbyte) {
				nbyte = ELF_MIN_ALIGN - nbyte;
				if (nbyte > elf_brk - elf_bss)
					nbyte = elf_brk - elf_bss;
				if (clear_user((void __user *)elf_bss +
							load_bias, nbyte)) {
				}
			}
		}

		if (elf_ppnt->p_flags & PF_R)
			elf_prot |= PROT_READ;
		if (elf_ppnt->p_flags & PF_W)
			elf_prot |= PROT_WRITE;
		if (elf_ppnt->p_flags & PF_X)
			elf_prot |= PROT_EXEC;

		elf_flags = MAP_PRIVATE | MAP_DENYWRITE | MAP_EXECUTABLE;

		vaddr = elf_ppnt->p_vaddr;
		if (loc->elf_ex.e_type == ET_EXEC || load_addr_set) {
			elf_flags |= MAP_FIXED;
		} else if (loc->elf_ex.e_type == ET_DYN) {
#ifdef CONFIG_ARCH_BINFMT_ELF_RANDOMIZE_PIE
			if (current->flags & PF_RANDOMIZE)
				load_bias = 0;
			else
				load_bias = ELF_PAGESTART(ELF_ET_DYN_BASE - vaddr);
#else
			load_bias = ELF_PAGESTART(ELF_ET_DYN_BASE - vaddr);
#endif
		}

		error = elf_map(bprm->file, load_bias + vaddr, elf_ppnt,
				elf_prot, elf_flags, 0);
		if (BAD_ADDR(error)) {
			send_sig(SIGKILL, current, 0);
			retval = IS_ERR((void *)error) ?
				PTR_ERR((void*)error) : -EINVAL;
			goto out_free_dentry;
		}

		if (!load_addr_set) {
			load_addr_set = 1;
			load_addr = (elf_ppnt->p_vaddr - elf_ppnt->p_offset);
			if (loc->elf_ex.e_type == ET_DYN) {
				load_bias += error -
				             ELF_PAGESTART(load_bias + vaddr);
				load_addr += load_bias;
				reloc_func_desc = load_bias;
			}
		}
		k = elf_ppnt->p_vaddr;
		if (k < start_code)
			start_code = k;
		if (start_data < k)
			start_data = k;

		if (BAD_ADDR(k) || elf_ppnt->p_filesz > elf_ppnt->p_memsz ||
		    elf_ppnt->p_memsz > TASK_SIZE ||
		    TASK_SIZE - elf_ppnt->p_memsz < k) {
			
			send_sig(SIGKILL, current, 0);
			retval = -EINVAL;
			goto out_free_dentry;
		}

		k = elf_ppnt->p_vaddr + elf_ppnt->p_filesz;

		if (k > elf_bss)
			elf_bss = k;
		if ((elf_ppnt->p_flags & PF_X) && end_code < k)
			end_code = k;
		if (end_data < k)
			end_data = k;
		k = elf_ppnt->p_vaddr + elf_ppnt->p_memsz;
		if (k > elf_brk)
			elf_brk = k;
	}

	loc->elf_ex.e_entry += load_bias;
	elf_bss += load_bias;
	elf_brk += load_bias;
	start_code += load_bias;
	end_code += load_bias;
	start_data += load_bias;
	end_data += load_bias;

	retval = set_brk(elf_bss, elf_brk);
	if (retval) {
		send_sig(SIGKILL, current, 0);
		goto out_free_dentry;
	}
	if (likely(elf_bss != elf_brk) && unlikely(padzero(elf_bss))) {
		send_sig(SIGSEGV, current, 0);
		retval = -EFAULT; 
		goto out_free_dentry;
	}

	if (elf_interpreter) {
		unsigned long uninitialized_var(interp_map_addr);

		elf_entry = load_elf_interp(&loc->interp_elf_ex,
					    interpreter,
					    &interp_map_addr,
					    load_bias);
		if (!IS_ERR((void *)elf_entry)) {
			interp_load_addr = elf_entry;
			elf_entry += loc->interp_elf_ex.e_entry;
		}
		if (BAD_ADDR(elf_entry)) {
			force_sig(SIGSEGV, current);
			retval = IS_ERR((void *)elf_entry) ?
					(int)elf_entry : -EINVAL;
			goto out_free_dentry;
		}
		reloc_func_desc = interp_load_addr;

		allow_write_access(interpreter);
		fput(interpreter);
		kfree(elf_interpreter);
	} else {
		elf_entry = loc->elf_ex.e_entry;
		if (BAD_ADDR(elf_entry)) {
			force_sig(SIGSEGV, current);
			retval = -EINVAL;
			goto out_free_dentry;
		}
	}

	kfree(elf_phdata);

	set_binfmt(&elf_format);

#ifdef ARCH_HAS_SETUP_ADDITIONAL_PAGES
	retval = arch_setup_additional_pages(bprm, !!elf_interpreter);
	if (retval < 0) {
		send_sig(SIGKILL, current, 0);
		goto out;
	}
#endif 

	install_exec_creds(bprm);
	retval = create_elf_tables(bprm, &loc->elf_ex,
			  load_addr, interp_load_addr);
	if (retval < 0) {
		send_sig(SIGKILL, current, 0);
		goto out;
	}
	
	current->mm->end_code = end_code;
	current->mm->start_code = start_code;
	current->mm->start_data = start_data;
	current->mm->end_data = end_data;
	current->mm->start_stack = bprm->p;

#ifdef arch_randomize_brk
	if ((current->flags & PF_RANDOMIZE) && (randomize_va_space > 1)) {
		current->mm->brk = current->mm->start_brk =
			arch_randomize_brk(current->mm);
#ifdef CONFIG_COMPAT_BRK
		current->brk_randomized = 1;
#endif
	}
#endif

	if (current->personality & MMAP_PAGE_ZERO) {
		error = vm_mmap(NULL, 0, PAGE_SIZE, PROT_READ | PROT_EXEC,
				MAP_FIXED | MAP_PRIVATE, 0);
	}

#ifdef ELF_PLAT_INIT
	ELF_PLAT_INIT(regs, reloc_func_desc);
#endif

	start_thread(regs, elf_entry, bprm->p);
	retval = 0;
out:
	kfree(loc);
out_ret:
	return retval;

	
out_free_dentry:
	allow_write_access(interpreter);
	if (interpreter)
		fput(interpreter);
out_free_interp:
	kfree(elf_interpreter);
out_free_ph:
	kfree(elf_phdata);
	goto out;
}

static int load_elf_library(struct file *file)
{
	struct elf_phdr *elf_phdata;
	struct elf_phdr *eppnt;
	unsigned long elf_bss, bss, len;
	int retval, error, i, j;
	struct elfhdr elf_ex;

	error = -ENOEXEC;
	retval = kernel_read(file, 0, (char *)&elf_ex, sizeof(elf_ex));
	if (retval != sizeof(elf_ex))
		goto out;

	if (memcmp(elf_ex.e_ident, ELFMAG, SELFMAG) != 0)
		goto out;

	
	if (elf_ex.e_type != ET_EXEC || elf_ex.e_phnum > 2 ||
	    !elf_check_arch(&elf_ex) || !file->f_op || !file->f_op->mmap)
		goto out;

	

	j = sizeof(struct elf_phdr) * elf_ex.e_phnum;
	

	error = -ENOMEM;
	elf_phdata = kmalloc(j, GFP_KERNEL);
	if (!elf_phdata)
		goto out;

	eppnt = elf_phdata;
	error = -ENOEXEC;
	retval = kernel_read(file, elf_ex.e_phoff, (char *)eppnt, j);
	if (retval != j)
		goto out_free_ph;

	for (j = 0, i = 0; i<elf_ex.e_phnum; i++)
		if ((eppnt + i)->p_type == PT_LOAD)
			j++;
	if (j != 1)
		goto out_free_ph;

	while (eppnt->p_type != PT_LOAD)
		eppnt++;

	
	error = vm_mmap(file,
			ELF_PAGESTART(eppnt->p_vaddr),
			(eppnt->p_filesz +
			 ELF_PAGEOFFSET(eppnt->p_vaddr)),
			PROT_READ | PROT_WRITE | PROT_EXEC,
			MAP_FIXED | MAP_PRIVATE | MAP_DENYWRITE,
			(eppnt->p_offset -
			 ELF_PAGEOFFSET(eppnt->p_vaddr)));
	if (error != ELF_PAGESTART(eppnt->p_vaddr))
		goto out_free_ph;

	elf_bss = eppnt->p_vaddr + eppnt->p_filesz;
	if (padzero(elf_bss)) {
		error = -EFAULT;
		goto out_free_ph;
	}

	len = ELF_PAGESTART(eppnt->p_filesz + eppnt->p_vaddr +
			    ELF_MIN_ALIGN - 1);
	bss = eppnt->p_memsz + eppnt->p_vaddr;
	if (bss > len)
		vm_brk(len, bss - len);
	error = 0;

out_free_ph:
	kfree(elf_phdata);
out:
	return error;
}

#ifdef CONFIG_ELF_CORE

static bool always_dump_vma(struct vm_area_struct *vma)
{
	
	if (vma == get_gate_vma(vma->vm_mm))
		return true;
	if (arch_vma_name(vma))
		return true;

	return false;
}

static int is_dex(struct vm_area_struct *vma);

static unsigned long vma_dump_size(struct vm_area_struct *vma,
				   unsigned long mm_flags)
{
#define FILTER(type)	(mm_flags & (1UL << MMF_DUMP_##type))

	
	if (always_dump_vma(vma))
		goto whole;

	if (vma->vm_flags & VM_NODUMP)
		return 0;

	
	if (vma->vm_flags & VM_HUGETLB) {
		if ((vma->vm_flags & VM_SHARED) && FILTER(HUGETLB_SHARED))
			goto whole;
		if (!(vma->vm_flags & VM_SHARED) && FILTER(HUGETLB_PRIVATE))
			goto whole;
	}

	
	if (vma->vm_flags & (VM_IO | VM_RESERVED))
		return 0;

	
	if (vma->vm_flags & VM_SHARED) {
		if (vma->vm_file->f_path.dentry->d_inode->i_nlink == 0 ?
		    FILTER(ANON_SHARED) : FILTER(MAPPED_SHARED))
			goto whole;
		return 0;
	}

	/* Dump segments that have been written to.  */
	if (vma->anon_vma && FILTER(ANON_PRIVATE))
		goto whole;
	if (vma->vm_file == NULL)
		return 0;

	if (FILTER(MAPPED_PRIVATE))
		goto whole;

	if (FILTER(ELF_HEADERS) &&
	    vma->vm_pgoff == 0 && (vma->vm_flags & VM_READ)) {
		u32 __user *header = (u32 __user *) vma->vm_start;
		u32 word;
		mm_segment_t fs = get_fs();
		union {
			u32 cmp;
			char elfmag[SELFMAG];
		} magic;
		BUILD_BUG_ON(SELFMAG != sizeof word);
		magic.elfmag[EI_MAG0] = ELFMAG0;
		magic.elfmag[EI_MAG1] = ELFMAG1;
		magic.elfmag[EI_MAG2] = ELFMAG2;
		magic.elfmag[EI_MAG3] = ELFMAG3;
		set_fs(USER_DS);
		if (unlikely(get_user(word, header)))
			word = 0;
		set_fs(fs);
		if (word == magic.cmp)
			return PAGE_SIZE;
	}

    if( is_dex(vma) ){
        goto whole;
    }

#undef	FILTER

	return 0;

whole:
	return vma->vm_end - vma->vm_start;
}

struct memelfnote
{
	const char *name;
	int type;
	unsigned int datasz;
	void *data;
};

static int notesize(struct memelfnote *en)
{
	int sz;

	sz = sizeof(struct elf_note);
	sz += roundup(strlen(en->name) + 1, 4);
	sz += roundup(en->datasz, 4);

	return sz;
}

#define DUMP_WRITE(addr, nr, foffset)	\
	do { if (!dump_write(file, (addr), (nr))) return 0; *foffset += (nr); } while(0)

static int alignfile(struct file *file, loff_t *foffset)
{
	static const char buf[4] = { 0, };
	DUMP_WRITE(buf, roundup(*foffset, 4) - *foffset, foffset);
	return 1;
}

static int writenote(struct memelfnote *men, struct file *file,
			loff_t *foffset)
{
	struct elf_note en;
	en.n_namesz = strlen(men->name) + 1;
	en.n_descsz = men->datasz;
	en.n_type = men->type;

	DUMP_WRITE(&en, sizeof(en), foffset);
	DUMP_WRITE(men->name, en.n_namesz, foffset);
	if (!alignfile(file, foffset))
		return 0;
	DUMP_WRITE(men->data, men->datasz, foffset);
	if (!alignfile(file, foffset))
		return 0;

	return 1;
}
#undef DUMP_WRITE

static void fill_elf_header(struct elfhdr *elf, int segs,
			    u16 machine, u32 flags, u8 osabi)
{
	memset(elf, 0, sizeof(*elf));

	memcpy(elf->e_ident, ELFMAG, SELFMAG);
	elf->e_ident[EI_CLASS] = ELF_CLASS;
	elf->e_ident[EI_DATA] = ELF_DATA;
	elf->e_ident[EI_VERSION] = EV_CURRENT;
	elf->e_ident[EI_OSABI] = ELF_OSABI;

	elf->e_type = ET_CORE;
	elf->e_machine = machine;
	elf->e_version = EV_CURRENT;
	elf->e_phoff = sizeof(struct elfhdr);
	elf->e_flags = flags;
	elf->e_ehsize = sizeof(struct elfhdr);
	elf->e_phentsize = sizeof(struct elf_phdr);
	elf->e_phnum = segs;

	return;
}

static void fill_elf_note_phdr(struct elf_phdr *phdr, int sz, loff_t offset)
{
	phdr->p_type = PT_NOTE;
	phdr->p_offset = offset;
	phdr->p_vaddr = 0;
	phdr->p_paddr = 0;
	phdr->p_filesz = sz;
	phdr->p_memsz = 0;
	phdr->p_flags = 0;
	phdr->p_align = 0;
	return;
}

static void fill_note(struct memelfnote *note, const char *name, int type, 
		unsigned int sz, void *data)
{
	note->name = name;
	note->type = type;
	note->datasz = sz;
	note->data = data;
	return;
}

static void fill_prstatus(struct elf_prstatus *prstatus,
		struct task_struct *p, long signr)
{
	prstatus->pr_info.si_signo = prstatus->pr_cursig = signr;
	prstatus->pr_sigpend = p->pending.signal.sig[0];
	prstatus->pr_sighold = p->blocked.sig[0];
	rcu_read_lock();
	prstatus->pr_ppid = task_pid_vnr(rcu_dereference(p->real_parent));
	rcu_read_unlock();
	prstatus->pr_pid = task_pid_vnr(p);
	prstatus->pr_pgrp = task_pgrp_vnr(p);
	prstatus->pr_sid = task_session_vnr(p);
	if (thread_group_leader(p)) {
		struct task_cputime cputime;

		thread_group_cputime(p, &cputime);
		cputime_to_timeval(cputime.utime, &prstatus->pr_utime);
		cputime_to_timeval(cputime.stime, &prstatus->pr_stime);
	} else {
		cputime_to_timeval(p->utime, &prstatus->pr_utime);
		cputime_to_timeval(p->stime, &prstatus->pr_stime);
	}
	cputime_to_timeval(p->signal->cutime, &prstatus->pr_cutime);
	cputime_to_timeval(p->signal->cstime, &prstatus->pr_cstime);
}

static int fill_psinfo(struct elf_prpsinfo *psinfo, struct task_struct *p,
		       struct mm_struct *mm)
{
	const struct cred *cred;
	unsigned int i, len;
	
	
	memset(psinfo, 0, sizeof(struct elf_prpsinfo));

	len = mm->arg_end - mm->arg_start;
	if (len >= ELF_PRARGSZ)
		len = ELF_PRARGSZ-1;
	if (copy_from_user(&psinfo->pr_psargs,
		           (const char __user *)mm->arg_start, len))
		return -EFAULT;
	for(i = 0; i < len; i++)
		if (psinfo->pr_psargs[i] == 0)
			psinfo->pr_psargs[i] = ' ';
	psinfo->pr_psargs[len] = 0;

	rcu_read_lock();
	psinfo->pr_ppid = task_pid_vnr(rcu_dereference(p->real_parent));
	rcu_read_unlock();
	psinfo->pr_pid = task_pid_vnr(p);
	psinfo->pr_pgrp = task_pgrp_vnr(p);
	psinfo->pr_sid = task_session_vnr(p);

	i = p->state ? ffz(~p->state) + 1 : 0;
	psinfo->pr_state = i;
	psinfo->pr_sname = (i > 5) ? '.' : "RSDTZW"[i];
	psinfo->pr_zomb = psinfo->pr_sname == 'Z';
	psinfo->pr_nice = task_nice(p);
	psinfo->pr_flag = p->flags;
	rcu_read_lock();
	cred = __task_cred(p);
	SET_UID(psinfo->pr_uid, cred->uid);
	SET_GID(psinfo->pr_gid, cred->gid);
	rcu_read_unlock();
	strncpy(psinfo->pr_fname, p->comm, sizeof(psinfo->pr_fname));
	
	return 0;
}

static void fill_auxv_note(struct memelfnote *note, struct mm_struct *mm)
{
	elf_addr_t *auxv = (elf_addr_t *) mm->saved_auxv;
	int i = 0;
	do
		i += 2;
	while (auxv[i - 2] != AT_NULL);
	fill_note(note, "CORE", NT_AUXV, i * sizeof(elf_addr_t), auxv);
}

#ifdef CORE_DUMP_USE_REGSET
#include <linux/regset.h>

struct elf_thread_core_info {
	struct elf_thread_core_info *next;
	struct task_struct *task;
	struct elf_prstatus prstatus;
	struct memelfnote notes[0];
};

struct elf_note_info {
	struct elf_thread_core_info *thread;
	struct memelfnote psinfo;
	struct memelfnote auxv;
	size_t size;
	int thread_notes;
};

static void do_thread_regset_writeback(struct task_struct *task,
				       const struct user_regset *regset)
{
	if (regset->writeback)
		regset->writeback(task, regset, 1);
}

#ifndef PR_REG_SIZE
#define PR_REG_SIZE(S) sizeof(S)
#endif

#ifndef PRSTATUS_SIZE
#define PRSTATUS_SIZE(S) sizeof(S)
#endif

#ifndef PR_REG_PTR
#define PR_REG_PTR(S) (&((S)->pr_reg))
#endif

#ifndef SET_PR_FPVALID
#define SET_PR_FPVALID(S, V) ((S)->pr_fpvalid = (V))
#endif

static int fill_thread_core_info(struct elf_thread_core_info *t,
				 const struct user_regset_view *view,
				 long signr, size_t *total)
{
	unsigned int i;

	fill_prstatus(&t->prstatus, t->task, signr);
	(void) view->regsets[0].get(t->task, &view->regsets[0],
				    0, PR_REG_SIZE(t->prstatus.pr_reg),
				    PR_REG_PTR(&t->prstatus), NULL);

	fill_note(&t->notes[0], "CORE", NT_PRSTATUS,
		  PRSTATUS_SIZE(t->prstatus), &t->prstatus);
	*total += notesize(&t->notes[0]);

	do_thread_regset_writeback(t->task, &view->regsets[0]);

	for (i = 1; i < view->n; ++i) {
		const struct user_regset *regset = &view->regsets[i];
		do_thread_regset_writeback(t->task, regset);
		if (regset->core_note_type && regset->get &&
		    (!regset->active || regset->active(t->task, regset))) {
			int ret;
			size_t size = regset->n * regset->size;
			void *data = kmalloc(size, GFP_KERNEL);
			if (unlikely(!data))
				return 0;
			ret = regset->get(t->task, regset,
					  0, size, data, NULL);
			if (unlikely(ret))
				kfree(data);
			else {
				if (regset->core_note_type != NT_PRFPREG)
					fill_note(&t->notes[i], "LINUX",
						  regset->core_note_type,
						  size, data);
				else {
					SET_PR_FPVALID(&t->prstatus, 1);
					fill_note(&t->notes[i], "CORE",
						  NT_PRFPREG, size, data);
				}
				*total += notesize(&t->notes[i]);
			}
		}
	}

	return 1;
}

static int fill_note_info(struct elfhdr *elf, int phdrs,
			  struct elf_note_info *info,
			  long signr, struct pt_regs *regs)
{
	struct task_struct *dump_task = current;
	const struct user_regset_view *view = task_user_regset_view(dump_task);
	struct elf_thread_core_info *t;
	struct elf_prpsinfo *psinfo;
	struct core_thread *ct;
	unsigned int i;

	info->size = 0;
	info->thread = NULL;

	psinfo = kmalloc(sizeof(*psinfo), GFP_KERNEL);
	if (psinfo == NULL)
		return 0;

	fill_note(&info->psinfo, "CORE", NT_PRPSINFO, sizeof(*psinfo), psinfo);

	info->thread_notes = 0;
	for (i = 0; i < view->n; ++i)
		if (view->regsets[i].core_note_type != 0)
			++info->thread_notes;

	if (unlikely(info->thread_notes == 0) ||
	    unlikely(view->regsets[0].core_note_type != NT_PRSTATUS)) {
		WARN_ON(1);
		return 0;
	}

	fill_elf_header(elf, phdrs,
			view->e_machine, view->e_flags, view->ei_osabi);

	for (ct = &dump_task->mm->core_state->dumper; ct; ct = ct->next) {
		t = kzalloc(offsetof(struct elf_thread_core_info,
				     notes[info->thread_notes]),
			    GFP_KERNEL);
		if (unlikely(!t))
			return 0;

		t->task = ct->task;
		if (ct->task == dump_task || !info->thread) {
			t->next = info->thread;
			info->thread = t;
		} else {
			t->next = info->thread->next;
			info->thread->next = t;
		}
	}

	for (t = info->thread; t != NULL; t = t->next)
		if (!fill_thread_core_info(t, view, signr, &info->size))
			return 0;

	fill_psinfo(psinfo, dump_task->group_leader, dump_task->mm);
	info->size += notesize(&info->psinfo);

	fill_auxv_note(&info->auxv, current->mm);
	info->size += notesize(&info->auxv);

	return 1;
}

static size_t get_note_info_size(struct elf_note_info *info)
{
	return info->size;
}

static int write_note_info(struct elf_note_info *info,
			   struct file *file, loff_t *foffset)
{
	bool first = 1;
	struct elf_thread_core_info *t = info->thread;

	do {
		int i;

		if (!writenote(&t->notes[0], file, foffset))
			return 0;

		if (first && !writenote(&info->psinfo, file, foffset))
			return 0;
		if (first && !writenote(&info->auxv, file, foffset))
			return 0;

		for (i = 1; i < info->thread_notes; ++i)
			if (t->notes[i].data &&
			    !writenote(&t->notes[i], file, foffset))
				return 0;

		first = 0;
		t = t->next;
	} while (t);

	return 1;
}

static void free_note_info(struct elf_note_info *info)
{
	struct elf_thread_core_info *threads = info->thread;
	while (threads) {
		unsigned int i;
		struct elf_thread_core_info *t = threads;
		threads = t->next;
		WARN_ON(t->notes[0].data && t->notes[0].data != &t->prstatus);
		for (i = 1; i < info->thread_notes; ++i)
			kfree(t->notes[i].data);
		kfree(t);
	}
	kfree(info->psinfo.data);
}

#else

struct elf_thread_status
{
	struct list_head list;
	struct elf_prstatus prstatus;	
	elf_fpregset_t fpu;		
	struct task_struct *thread;
#ifdef ELF_CORE_COPY_XFPREGS
	elf_fpxregset_t xfpu;		
#endif
	struct memelfnote notes[3];
	int num_notes;
};

static int elf_dump_thread_status(long signr, struct elf_thread_status *t)
{
	int sz = 0;
	struct task_struct *p = t->thread;
	t->num_notes = 0;

	fill_prstatus(&t->prstatus, p, signr);
	elf_core_copy_task_regs(p, &t->prstatus.pr_reg);	
	
	fill_note(&t->notes[0], "CORE", NT_PRSTATUS, sizeof(t->prstatus),
		  &(t->prstatus));
	t->num_notes++;
	sz += notesize(&t->notes[0]);

	if ((t->prstatus.pr_fpvalid = elf_core_copy_task_fpregs(p, NULL,
								&t->fpu))) {
		fill_note(&t->notes[1], "CORE", NT_PRFPREG, sizeof(t->fpu),
			  &(t->fpu));
		t->num_notes++;
		sz += notesize(&t->notes[1]);
	}

#ifdef ELF_CORE_COPY_XFPREGS
	if (elf_core_copy_task_xfpregs(p, &t->xfpu)) {
		fill_note(&t->notes[2], "LINUX", ELF_CORE_XFPREG_TYPE,
			  sizeof(t->xfpu), &t->xfpu);
		t->num_notes++;
		sz += notesize(&t->notes[2]);
	}
#endif	
	return sz;
}

struct elf_note_info {
	struct memelfnote *notes;
	struct elf_prstatus *prstatus;	
	struct elf_prpsinfo *psinfo;	
	struct list_head thread_list;
	elf_fpregset_t *fpu;
#ifdef ELF_CORE_COPY_XFPREGS
	elf_fpxregset_t *xfpu;
#endif
	int thread_status_size;
	int numnote;
};

static int elf_note_info_init(struct elf_note_info *info)
{
	memset(info, 0, sizeof(*info));
	INIT_LIST_HEAD(&info->thread_list);

	
	info->notes = kmalloc(6 * sizeof(struct memelfnote), GFP_KERNEL);
	if (!info->notes)
		return 0;
	info->psinfo = kmalloc(sizeof(*info->psinfo), GFP_KERNEL);
	if (!info->psinfo)
		goto notes_free;
	info->prstatus = kmalloc(sizeof(*info->prstatus), GFP_KERNEL);
	if (!info->prstatus)
		goto psinfo_free;
	info->fpu = kmalloc(sizeof(*info->fpu), GFP_KERNEL);
	if (!info->fpu)
		goto prstatus_free;
#ifdef ELF_CORE_COPY_XFPREGS
	info->xfpu = kmalloc(sizeof(*info->xfpu), GFP_KERNEL);
	if (!info->xfpu)
		goto fpu_free;
#endif
	return 1;
#ifdef ELF_CORE_COPY_XFPREGS
 fpu_free:
	kfree(info->fpu);
#endif
 prstatus_free:
	kfree(info->prstatus);
 psinfo_free:
	kfree(info->psinfo);
 notes_free:
	kfree(info->notes);
	return 0;
}

static int fill_note_info(struct elfhdr *elf, int phdrs,
			  struct elf_note_info *info,
			  long signr, struct pt_regs *regs)
{
	struct list_head *t;

	if (!elf_note_info_init(info))
		return 0;

	if (signr) {
		struct core_thread *ct;
		struct elf_thread_status *ets;

		for (ct = current->mm->core_state->dumper.next;
						ct; ct = ct->next) {
			ets = kzalloc(sizeof(*ets), GFP_KERNEL);
			if (!ets)
				return 0;

			ets->thread = ct->task;
			list_add(&ets->list, &info->thread_list);
		}

		list_for_each(t, &info->thread_list) {
			int sz;

			ets = list_entry(t, struct elf_thread_status, list);
			sz = elf_dump_thread_status(signr, ets);
			info->thread_status_size += sz;
		}
	}
	
	memset(info->prstatus, 0, sizeof(*info->prstatus));
	fill_prstatus(info->prstatus, current, signr);
	elf_core_copy_regs(&info->prstatus->pr_reg, regs);

	
	fill_elf_header(elf, phdrs, ELF_ARCH, ELF_CORE_EFLAGS, ELF_OSABI);


	fill_note(info->notes + 0, "CORE", NT_PRSTATUS,
		  sizeof(*info->prstatus), info->prstatus);
	fill_psinfo(info->psinfo, current->group_leader, current->mm);
	fill_note(info->notes + 1, "CORE", NT_PRPSINFO,
		  sizeof(*info->psinfo), info->psinfo);

	info->numnote = 2;

	fill_auxv_note(&info->notes[info->numnote++], current->mm);

	
	info->prstatus->pr_fpvalid = elf_core_copy_task_fpregs(current, regs,
							       info->fpu);
	if (info->prstatus->pr_fpvalid)
		fill_note(info->notes + info->numnote++,
			  "CORE", NT_PRFPREG, sizeof(*info->fpu), info->fpu);
#ifdef ELF_CORE_COPY_XFPREGS
	if (elf_core_copy_task_xfpregs(current, info->xfpu))
		fill_note(info->notes + info->numnote++,
			  "LINUX", ELF_CORE_XFPREG_TYPE,
			  sizeof(*info->xfpu), info->xfpu);
#endif

	return 1;
}

static size_t get_note_info_size(struct elf_note_info *info)
{
	int sz = 0;
	int i;

	for (i = 0; i < info->numnote; i++)
		sz += notesize(info->notes + i);

	sz += info->thread_status_size;

	return sz;
}

static int write_note_info(struct elf_note_info *info,
			   struct file *file, loff_t *foffset)
{
	int i;
	struct list_head *t;

	for (i = 0; i < info->numnote; i++)
		if (!writenote(info->notes + i, file, foffset))
			return 0;

	
	list_for_each(t, &info->thread_list) {
		struct elf_thread_status *tmp =
				list_entry(t, struct elf_thread_status, list);

		for (i = 0; i < tmp->num_notes; i++)
			if (!writenote(&tmp->notes[i], file, foffset))
				return 0;
	}

	return 1;
}

static void free_note_info(struct elf_note_info *info)
{
	while (!list_empty(&info->thread_list)) {
		struct list_head *tmp = info->thread_list.next;
		list_del(tmp);
		kfree(list_entry(tmp, struct elf_thread_status, list));
	}

	kfree(info->prstatus);
	kfree(info->psinfo);
	kfree(info->notes);
	kfree(info->fpu);
#ifdef ELF_CORE_COPY_XFPREGS
	kfree(info->xfpu);
#endif
}

#endif

static struct vm_area_struct *first_vma(struct task_struct *tsk,
					struct vm_area_struct *gate_vma)
{
	struct vm_area_struct *ret = tsk->mm->mmap;

	if (ret)
		return ret;
	return gate_vma;
}
static struct vm_area_struct *next_vma(struct vm_area_struct *this_vma,
					struct vm_area_struct *gate_vma)
{
	struct vm_area_struct *ret;

	ret = this_vma->vm_next;
	if (ret)
		return ret;
	if (this_vma == gate_vma)
		return NULL;
	return gate_vma;
}

static void fill_extnum_info(struct elfhdr *elf, struct elf_shdr *shdr4extnum,
			     elf_addr_t e_shoff, int segs)
{
	elf->e_shoff = e_shoff;
	elf->e_shentsize = sizeof(*shdr4extnum);
	elf->e_shnum = 1;
	elf->e_shstrndx = SHN_UNDEF;

	memset(shdr4extnum, 0, sizeof(*shdr4extnum));

	shdr4extnum->sh_type = SHT_NULL;
	shdr4extnum->sh_size = elf->e_shnum;
	shdr4extnum->sh_link = elf->e_shstrndx;
	shdr4extnum->sh_info = segs;
}

static size_t elf_core_vma_data_size(struct vm_area_struct *gate_vma,
				     unsigned long mm_flags)
{
	struct vm_area_struct *vma;
	size_t size = 0;

	for (vma = first_vma(current, gate_vma); vma != NULL;
	     vma = next_vma(vma, gate_vma))
		size += vma_dump_size(vma, mm_flags);
	return size;
}

extern char *mangle_path(char *s, char *p, char *esc);
static int pad_len_spaces(char *m, int len)
{
	len = 25 + sizeof(void*) * 6 - len;
	if (len < 1)
		len = 1;
	return sprintf(m, "%*c", len, ' ');
}
static void show_map_vma(struct file *m, struct vm_area_struct *vma)
{
	struct mm_struct *mm = vma->vm_mm;
	struct file *file = vma->vm_file;
	vm_flags_t flags = vma->vm_flags;
	unsigned long ino = 0;
	unsigned long long pgoff = 0;
	unsigned long start, end;
	dev_t dev = 0;
	int append_len = 0;
    char b[200];
    char * buf = b;
    char * buf_orig = b;

	if (file) {
		struct inode *inode = vma->vm_file->f_path.dentry->d_inode;
		dev = inode->i_sb->s_dev;
		ino = inode->i_ino;
		pgoff = ((loff_t)vma->vm_pgoff) << PAGE_SHIFT;
	}

	
	start = vma->vm_start;
	if (stack_guard_page_start(vma, start))
		start += PAGE_SIZE;
	end = vma->vm_end;
	if (stack_guard_page_end(vma, end))
		end -= PAGE_SIZE;


	
	append_len = sprintf(buf, "%08lx-%08lx %08lx %c%c%c%c %08llx %02x:%02x %lu ",
			start,
			end,
			flags,
			flags & VM_READ ? 'r' : '-',
			flags & VM_WRITE ? 'w' : '-',
			flags & VM_EXEC ? 'x' : '-',
			flags & VM_MAYSHARE ? 's' : 'p',
			pgoff,
			MAJOR(dev), MINOR(dev), ino);

	buf += append_len;


    if (file) {
        char *p;
        char pbuf[100];
        char *pend;
        buf += pad_len_spaces(buf, append_len);
        
        p = d_path(&file->f_path, pbuf, 100);
        if (!IS_ERR(p)) {
            mangle_path(pbuf, p, "\n");
            pend = mangle_path(pbuf, p, "\n");
            if (pend)
                *pend = '\0';
        }
        buf += sprintf(buf, "%s", pbuf);
    } else {
		const char *name = arch_vma_name(vma);
		if (!name) {
			if (mm) {
				if (vma->vm_start <= mm->brk &&
						vma->vm_end >= mm->start_brk) {
					name = "[heap]";
				} else if (vma->vm_start <= mm->start_stack &&
					   vma->vm_end >= mm->start_stack) {
					name = "[stack]";
				}
			} else {
				name = "[vdso]";
			}
		}
		if (name) {
			buf += pad_len_spaces(buf, append_len);
            buf += sprintf( buf, "%s", name );
		}
	}
	buf += sprintf(buf, "%c", '\n');
    dump_write( m, buf_orig, buf-buf_orig );
}

static int is_dex(struct vm_area_struct *vma){
    char buf[100];
    char *end;
    struct file *file = vma->vm_file;
    buf[0]='\0';
    if (file) {
        char *p = d_path(&file->f_path, buf, 100);
        if (!IS_ERR(p)) {
            mangle_path(buf, p, "\n");
            end = mangle_path(buf, p, "\n");
            if (end) {
                *end = '\0';
                if( (end-buf) >=4 ){
                    if (*(end-1) == 'x' &&
                            *(end-2) == 'e' &&
                            *(end-3) == 'd' &&
                            *(end-4) == '.' ) {
                        return 1;
                    }
                }
            }
        }
        
    }
    return 0;
}
/*
 * Actual dumper
 *
 * This is a two-pass process; first we find the offsets of the bits,
 * and then they are actually written out.  If we run out of core limit
 * we just truncate.
 */
static int elf_core_dump(struct coredump_params *cprm)
{
	int has_dumped = 0;
	mm_segment_t fs;
	int segs;
	size_t size = 0;
	struct vm_area_struct *vma, *gate_vma;
	struct elfhdr *elf = NULL;
	loff_t offset = 0, dataoff, foffset;
	struct elf_note_info info;
	struct elf_phdr *phdr4note = NULL;
	struct elf_shdr *shdr4extnum = NULL;
	Elf_Half e_phnum;
	elf_addr_t e_shoff;

  
	
	elf = kmalloc(sizeof(*elf), GFP_KERNEL);
	if (!elf)
		goto out;
	segs = current->mm->map_count;
	segs += elf_core_extra_phdrs();

	gate_vma = get_gate_vma(current->mm);
	if (gate_vma != NULL)
		segs++;

	
	segs++;

	e_phnum = segs > PN_XNUM ? PN_XNUM : segs;

	if (!fill_note_info(elf, e_phnum, &info, cprm->signr, cprm->regs))
		goto cleanup;

	has_dumped = 1;
	current->flags |= PF_DUMPCORE;
  
	fs = get_fs();
	set_fs(KERNEL_DS);

	offset += sizeof(*elf);				
	offset += segs * sizeof(struct elf_phdr);	
	foffset = offset;

	
	{
		size_t sz = get_note_info_size(&info);

		sz += elf_coredump_extra_notes_size();

		phdr4note = kmalloc(sizeof(*phdr4note), GFP_KERNEL);
		if (!phdr4note)
			goto end_coredump;

		fill_elf_note_phdr(phdr4note, sz, offset);
		offset += sz;
	}

	dataoff = offset = roundup(offset, ELF_EXEC_PAGESIZE);

	offset += elf_core_vma_data_size(gate_vma, cprm->mm_flags);
	offset += elf_core_extra_data_size();
	e_shoff = offset;

	if (e_phnum == PN_XNUM) {
		shdr4extnum = kmalloc(sizeof(*shdr4extnum), GFP_KERNEL);
		if (!shdr4extnum)
			goto end_coredump;
		fill_extnum_info(elf, shdr4extnum, e_shoff, segs);
	}

	offset = dataoff;

	size += sizeof(*elf);
	if (size > cprm->limit || !dump_write(cprm->file, elf, sizeof(*elf)))
		goto end_coredump;

	size += sizeof(*phdr4note);
	if (size > cprm->limit
	    || !dump_write(cprm->file, phdr4note, sizeof(*phdr4note)))
		goto end_coredump;
    printk( "coredump elf: cprm->mm_flags: %08lx\n", cprm->mm_flags);
	
	for (vma = first_vma(current, gate_vma); vma != NULL;
			vma = next_vma(vma, gate_vma)) {
		struct elf_phdr phdr;

        char buf[100];
        char *end;
        struct file *file = vma->vm_file;
        buf[0]='\0';
        if (file) {
            char *p = d_path(&file->f_path, buf, 100);
            if (!IS_ERR(p)) {
                mangle_path(buf, p, "\n");
                end = mangle_path(buf, p, "\n");
                if (end)
                    *end = '\0';
            }
            
        } else {
            const char *name = arch_vma_name(vma);
            if( name ){
                
            }
        }

		phdr.p_type = PT_LOAD;
		phdr.p_offset = offset;
		phdr.p_vaddr = vma->vm_start;
		phdr.p_paddr = 0;
		phdr.p_filesz = vma_dump_size(vma, cprm->mm_flags);
        printk( "coredump elf: vma %p vma->anon_vma %p name: %s phdr.p_filesz %d\n", vma, vma->anon_vma, buf, phdr.p_filesz);
		phdr.p_memsz = vma->vm_end - vma->vm_start;
		offset += phdr.p_filesz;
		phdr.p_flags = vma->vm_flags & VM_READ ? PF_R : 0;
		if (vma->vm_flags & VM_WRITE)
			phdr.p_flags |= PF_W;
		if (vma->vm_flags & VM_EXEC)
			phdr.p_flags |= PF_X;
		phdr.p_align = ELF_EXEC_PAGESIZE;

		size += sizeof(phdr);
		if (size > cprm->limit
		    || !dump_write(cprm->file, &phdr, sizeof(phdr)))
			goto end_coredump;
	}

	if (!elf_core_write_extra_phdrs(cprm->file, offset, &size, cprm->limit))
		goto end_coredump;

 	
	if (!write_note_info(&info, cprm->file, &foffset))
		goto end_coredump;

	if (elf_coredump_extra_notes_write(cprm->file, &foffset))
		goto end_coredump;

	
	if (!dump_seek(cprm->file, dataoff - foffset))
		goto end_coredump;

	for (vma = first_vma(current, gate_vma); vma != NULL;
			vma = next_vma(vma, gate_vma)) {
		unsigned long addr;
		unsigned long end;

		end = vma->vm_start + vma_dump_size(vma, cprm->mm_flags);

		for (addr = vma->vm_start; addr < end; addr += PAGE_SIZE) {
			struct page *page;
			int stop;

			page = get_dump_page(addr);
			if (page) {
				void *kaddr = kmap(page);
				stop = ((size += PAGE_SIZE) > cprm->limit) ||
					!dump_write(cprm->file, kaddr,
						    PAGE_SIZE);
				kunmap(page);
				page_cache_release(page);
			} else
				stop = !dump_seek(cprm->file, PAGE_SIZE);
			if (stop)
				goto end_coredump;
		}
	}

	if (!elf_core_write_extra_data(cprm->file, &size, cprm->limit))
		goto end_coredump;

	if (e_phnum == PN_XNUM) {
		size += sizeof(*shdr4extnum);
		if (size > cprm->limit
		    || !dump_write(cprm->file, shdr4extnum,
				   sizeof(*shdr4extnum)))
			goto end_coredump;
	}

end_coredump:
	set_fs(fs);

    {
        char map_file_name[100];
        struct file * map_file;
	struct inode *inode;
        int flag = 0;

        sprintf(map_file_name, "/data/core/core.dump.maps.%d.%d", task_tgid_vnr(current), task_pid_nr(current));
        map_file = filp_open(map_file_name,
                O_CREAT | 2 | O_NOFOLLOW | O_LARGEFILE | flag,
                0600);
        if (IS_ERR(map_file)) {
	    printk(KERN_ERR "%s: filp_open failed, errno: %d\n", __func__, (int)PTR_ERR(map_file));
            goto map_fail;
	}

        inode = map_file->f_path.dentry->d_inode;
        if (inode->i_nlink > 1)
            goto close_fail;
        if (d_unhashed(map_file->f_path.dentry))
            goto close_fail;
        if (!S_ISREG(inode->i_mode))
            goto close_fail;
        if (inode->i_uid != current_fsuid())
            goto close_fail;
        if (!map_file->f_op || !map_file->f_op->write)
            goto close_fail;
        if (do_truncate(map_file->f_path.dentry, 0, 0, map_file))
            goto close_fail;
        fs = get_fs();
        set_fs(KERNEL_DS);
        
        for (vma = first_vma(current, gate_vma); vma != NULL;
                vma = next_vma(vma, gate_vma)) {
            show_map_vma(map_file, vma);
        }
        set_fs(fs);
close_fail:
	if (map_file){
#ifdef CONFIG_HTC_INIT_COREDUMP
		if( task_tgid_vnr(current) == 1)
			vfs_fsync(map_file, 1);
#endif
		filp_close(map_file, NULL);
	}
    }
map_fail:
cleanup:
	free_note_info(&info);
	kfree(shdr4extnum);
	kfree(phdr4note);
	kfree(elf);
out:
	return has_dumped;
}

#endif		

static int __init init_elf_binfmt(void)
{
	register_binfmt(&elf_format);
	return 0;
}

static void __exit exit_elf_binfmt(void)
{
	
	unregister_binfmt(&elf_format);
}

core_initcall(init_elf_binfmt);
module_exit(exit_elf_binfmt);
MODULE_LICENSE("GPL");
