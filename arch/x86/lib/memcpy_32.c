// SPDX-License-Identifier: GPL-2.0
#include <linux/string.h>
#include <linux/export.h>

#undef memcpy
#undef memset

__visible void *memcpy(void *to, const void *from, size_t n)
{
#if defined(CONFIG_X86_USE_3DNOW) && !defined(CONFIG_FORTIFY_SOURCE)
	return __memcpy3d(to, from, n);
#else
	return __memcpy(to, from, n);
#endif
}
EXPORT_SYMBOL(memcpy);

__visible void *memset(void *s, int c, size_t count)
{
	return __memset(s, c, count);
}
EXPORT_SYMBOL(memset);
