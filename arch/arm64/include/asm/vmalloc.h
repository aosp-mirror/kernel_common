#ifndef _ASM_ARM64_VMALLOC_H
#define _ASM_ARM64_VMALLOC_H

#include <asm/page.h>
#include <asm/pgtable.h>

#define arch_vmap_pgprot_tagged arch_vmap_pgprot_tagged
static inline pgprot_t arch_vmap_pgprot_tagged(pgprot_t prot)
{
	return pgprot_tagged(prot);
}

#endif /* _ASM_ARM64_VMALLOC_H */
