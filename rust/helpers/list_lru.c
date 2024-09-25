// SPDX-License-Identifier: GPL-2.0

#include <linux/list_lru.h>

unsigned long rust_helper_list_lru_count(struct list_lru *lru)
{
	return list_lru_count(lru);
}

unsigned long rust_helper_list_lru_walk(struct list_lru *lru,
					list_lru_walk_cb isolate, void *cb_arg,
					unsigned long nr_to_walk)
{
	return list_lru_walk(lru, isolate, cb_arg, nr_to_walk);
}
