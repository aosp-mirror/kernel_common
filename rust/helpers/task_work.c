// SPDX-License-Identifier: GPL-2.0

#include <linux/task_work.h>

void rust_helper_init_task_work(struct callback_head *twork,
				task_work_func_t func)
{
	init_task_work(twork, func);
}
