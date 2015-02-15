#include <linux/kallsyms.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/of_fdt.h>

#include <asm/sections.h>
#include <asm/io.h>

#define KALLSYMS_MAGIC			(0xA0B1C2D3)

/*
 * the structure of `kallsyms_data' must be consisent with bootloader
 */
struct kallsyms_data {
	unsigned int magic;
	unsigned int addresses;
	unsigned int names;
	unsigned int num_syms;
	unsigned int token_table;
	unsigned int token_index;
	unsigned int markers;
	unsigned int stext;
	unsigned int sinittext;
	unsigned int einittext;
	unsigned int end;
};

/*
 * export the nessararily symbols from kallsyms.c
 */
extern const unsigned long kallsyms_addresses[] __attribute__((weak));
extern const u8 kallsyms_names[] __attribute__((weak));
extern const unsigned long kallsyms_num_syms
				__attribute__((weak, section(".rodata")));
extern const u8 kallsyms_token_table[] __attribute__((weak));
extern const u16 kallsyms_token_index[] __attribute__((weak));
extern const unsigned long kallsyms_markers[] __attribute__((weak));

struct htc_debug_info_data {
	unsigned int start;
	unsigned int size;
};

static int __init dt_scan_htc_debug_info(unsigned long node, const char *uname,
				     int depth, void *data)
{
	unsigned long l;
	char *p;

	pr_debug("search \"htc_debug_info\", depth: %d, uname: %s\n", depth, uname);

	if (depth != 1 || !data ||
	    (strcmp(uname, "htc_debug_info") != 0 && strcmp(uname, "htc_debug_info@0") != 0))
		return 0;

	pr_info("found \"htc_debug_info\"\n");
	p = of_get_flat_dt_prop(node, "kallsyms", &l);
	if (p != NULL && l > 0) {
		pr_info("found \"kallsyms\": loc: %x, size: %d, write to: %x\n",
				(unsigned int) p,
				min((size_t)l, sizeof(struct htc_debug_info_data)),
				(unsigned int) data);
		memcpy(data, p, min((size_t)l, sizeof(struct htc_debug_info_data)));
	}

	return 1;
}

static int __init kallsyms_addr_export(void)
{
	struct htc_debug_info_data info;
	struct kallsyms_data* data;

	/* Retrieve various information from the /htc_debug_info node */
	of_scan_flat_dt(dt_scan_htc_debug_info, &info);
	pr_debug("%s: &info: %08x\n",       __func__, (unsigned int) &info);
	pr_debug("%s: info->start: %08x\n", __func__, info.start);
	pr_debug("%s: info->size:  %08x\n", __func__, info.size);

	if (info.size < sizeof(*data)) {
		pr_err("%s: info->size (%08x) is not enough for kallsyms_data (%08x)\n",
				__func__, info.size, sizeof(struct kallsyms_data));
		return -1;
	}

	data = ioremap(info.start, info.size);
	if (!data) {
		pr_err("%s: fail to map htc_debug_info_data (start=%08x, size=%08x)\n",
				__func__, info.start, info.size);
		return -1;
	}

	data->magic       = (unsigned int) KALLSYMS_MAGIC;
	data->stext       = (unsigned int) _stext;
	data->sinittext   = (unsigned int) _sinittext;
	data->einittext   = (unsigned int) _einittext;
	data->end         = (unsigned int) _end;
	data->num_syms    = (unsigned int) kallsyms_num_syms;
	data->addresses   = (unsigned int) __pa(kallsyms_addresses);
	data->names       = (unsigned int) __pa(kallsyms_names);
	data->token_table = (unsigned int) __pa(kallsyms_token_table);
	data->token_index = (unsigned int) __pa(kallsyms_token_index);
	data->markers     = (unsigned int) __pa(kallsyms_markers);
	iounmap(data);

	return 0;
}
device_initcall(kallsyms_addr_export);
