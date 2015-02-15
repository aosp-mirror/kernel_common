#undef TRACE_SYSTEM
#define TRACE_SYSTEM f2fs

#if !defined(_TRACE_F2FS_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_F2FS_H

#include <linux/tracepoint.h>

DECLARE_EVENT_CLASS(f2fs_op,
		TP_PROTO(char *print_info),

		TP_ARGS(print_info),

		TP_STRUCT__entry(
			__string(print_info, print_info)
		),

		TP_fast_assign(
			__assign_str(print_info, print_info);
		),

		TP_printk("%s",
			__get_str(print_info)
		)
);

DEFINE_EVENT(f2fs_op, f2fs_write_begin,

	TP_PROTO(char *print_info),

	TP_ARGS(print_info)
);

DEFINE_EVENT(f2fs_op, f2fs_write_end,

		TP_PROTO(char *print_info),

		TP_ARGS(print_info)
);

DEFINE_EVENT(f2fs_op, f2fs_sync_file_enter,

		TP_PROTO(char *print_info),

		TP_ARGS(print_info)
);

DEFINE_EVENT(f2fs_op, f2fs_sync_file_exit,

		TP_PROTO(char *print_info),

		TP_ARGS(print_info)
);



#endif 

 
#include <trace/define_trace.h>
