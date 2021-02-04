/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2022 Google, Inc.
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM trusty

#if !defined(_TRUSTY_IPC_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRUSTY_IPC_TRACE_H

#include <linux/tracepoint.h>
#include <uapi/linux/trusty/ipc.h>
#include <linux/trusty/trusty_ipc.h>

/* One of the requirements of checkpatch.pl script is to have "parentheses
 * to sheild macro arguments from the effects of operator precedence."
 * Otherwise checkpatch.pl script shall throw below checkpatch error
 *
 * "ERROR: Macros with complex values should be enclosed in parentheses"
 *
 * Hence extra parenthese are used to avoid the above checkpatch error.
 * And the extra paentheses are removed using DELETE_PAREN to avoid
 * compilation errors.
 */
#define _DELETE_PAREN(args...)	args
#define DELETE_PAREN(arg)	_DELETE_PAREN(_DELETE_PAREN arg)

#define TIPC_CHANNEL_STATE_LIST (	\
	tipc_state(DISCONNECTED)	\
	tipc_state(CONNECTING)		\
	tipc_state(CONNECTED)		\
	tipc_state_end(STALE)		\
	)

#undef tipc_state
#undef tipc_state_end

#define tipc_state_define_enum(x)	(TRACE_DEFINE_ENUM(TIPC_##x);)
#define tipc_state(x)			DELETE_PAREN(tipc_state_define_enum(x))
#define tipc_state_end(x)		DELETE_PAREN(tipc_state_define_enum(x))

DELETE_PAREN(TIPC_CHANNEL_STATE_LIST)

#undef tipc_state
#undef tipc_state_end

#define tipc_state(x)		{ TIPC_##x, #x },
#define tipc_state_end(x)	{ TIPC_##x, #x }

#define tipc_channel_state_name(x)	\
	__print_symbolic(x, DELETE_PAREN(TIPC_CHANNEL_STATE_LIST))

TRACE_EVENT(trusty_ipc_connect,
	TP_PROTO(struct tipc_chan *chan, const char *port),
	TP_ARGS(chan, port),
	TP_STRUCT__entry(
		__field(u32, chan)
		__string(port, port)
		__field(int, state)
	),
	TP_fast_assign(
		__entry->chan = chan ? chan->local : ~0U;
		__assign_str(port, port);
		__entry->state = chan ? chan->state : 0;
	),
	TP_printk("chan=%u port=%s state=%s", __entry->chan, __get_str(port),
		tipc_channel_state_name(__entry->state))
);

TRACE_EVENT(trusty_ipc_connect_end,
	TP_PROTO(struct tipc_chan *chan, int err),
	TP_ARGS(chan, err),
	TP_STRUCT__entry(
		__field(u32, chan)
		__field(int, err)
		__field(int, state)
	),
	TP_fast_assign(
		__entry->chan = chan ? chan->local : ~0U;
		__entry->err = err;
		__entry->state = chan ? chan->state : 0;
	),
	TP_printk("chan=%u err=%d state=%s", __entry->chan, __entry->err,
		tipc_channel_state_name(__entry->state))
);

#define TIPC_CHANNEL_EVENT_LIST (	\
	tipc_event(CONNECTED)		\
	tipc_event(DISCONNECTED)	\
	tipc_event_end(SHUTDOWN)	\
	)

#undef tipc_event
#undef tipc_event_end

#define tipc_event_define_enum(x)	(TRACE_DEFINE_ENUM(TIPC_CHANNEL_##x);)
#define tipc_event(x)			DELETE_PAREN(tipc_event_define_enum(x))
#define tipc_event_end(x)		DELETE_PAREN(tipc_event_define_enum(x))

DELETE_PAREN(TIPC_CHANNEL_EVENT_LIST)

#undef tipc_event
#undef tipc_event_end

#define tipc_event(x)		{ TIPC_CHANNEL_##x, #x },
#define tipc_event_end(x)	{ TIPC_CHANNEL_##x, #x }

#define tipc_channel_event_name(x)	\
	__print_symbolic(x, DELETE_PAREN(TIPC_CHANNEL_EVENT_LIST))

TRACE_EVENT(trusty_ipc_handle_event,
	TP_PROTO(struct tipc_chan *chan, u32 event_id),
	TP_ARGS(chan, event_id),
	TP_STRUCT__entry(
		__field(u32, chan)
		__array(char, srv_name, MAX_SRV_NAME_LEN)
		__field(u32, event_id)
	),
	TP_fast_assign(
		__entry->chan = chan ? chan->local : ~0U;
		memcpy(__entry->srv_name, chan ? chan->srv_name : "", MAX_SRV_NAME_LEN);
		__entry->event_id = event_id;
	),
	TP_printk("chan=%u srv_name=%s event=%s", __entry->chan, __entry->srv_name,
		tipc_channel_event_name(__entry->event_id))
);

TRACE_EVENT(trusty_ipc_write,
	TP_PROTO(struct tipc_chan *chan,
		int len_or_err,
		struct tipc_msg_buf *txbuf,
		struct trusty_shm *shm),
	TP_ARGS(chan, len_or_err, txbuf, shm),
	TP_STRUCT__entry(
		__field(int, len_or_err)
		__field(u32, chan)
		__array(char, srv_name, MAX_SRV_NAME_LEN)
		__field(u64, buf_id)
		__field(size_t, shm_cnt)
		__dynamic_array(int, kind_shm, txbuf ? txbuf->shm_cnt : 0)
	),
	TP_fast_assign(
		size_t x;

		__entry->len_or_err = len_or_err;
		__entry->chan = chan ? chan->local : ~0U;
		memcpy(__entry->srv_name, chan ? chan->srv_name : "", MAX_SRV_NAME_LEN);
		__entry->buf_id = txbuf ? txbuf->buf_id : ~0ULL;
		__entry->shm_cnt = txbuf ? txbuf->shm_cnt : 0;
		if (shm) {
			for (x = 0; x < __entry->shm_cnt; x++)
				*((int *)__get_dynamic_array(kind_shm) + x) = shm[x].transfer;
		}
	),
	TP_printk("len_or_err=%d chan=%u srv_name=%s buf_id=0x%llx shm_cnt=%zu kind_shm=%s",
		__entry->len_or_err, __entry->chan, __entry->srv_name, __entry->buf_id,
		__entry->shm_cnt, __print_array(__get_dynamic_array(kind_shm),
		__get_dynamic_array_len(kind_shm) / sizeof(int), sizeof(int)))
);

TRACE_EVENT(trusty_ipc_read,
	TP_PROTO(struct tipc_chan *chan),
	TP_ARGS(chan),
	TP_STRUCT__entry(
		__field(u32, chan)
		__array(char, srv_name, MAX_SRV_NAME_LEN)
	),
	TP_fast_assign(
		__entry->chan = chan ? chan->local : ~0U;
		memcpy(__entry->srv_name, chan ? chan->srv_name : "", MAX_SRV_NAME_LEN);
	),
	TP_printk("chan=%u srv_name=%s", __entry->chan, __entry->srv_name)
);

TRACE_EVENT(trusty_ipc_read_end,
	TP_PROTO(struct tipc_chan *chan,
		int len_or_err,
		struct tipc_msg_buf *rxbuf),
	TP_ARGS(chan, len_or_err, rxbuf),
	TP_STRUCT__entry(
		__field(int, len_or_err)
		__field(u32, chan)
		__array(char, srv_name, MAX_SRV_NAME_LEN)
		__field(u64, buf_id)
		__field(size_t, shm_cnt)
	),
	TP_fast_assign(
		__entry->len_or_err = len_or_err;
		__entry->chan = chan ? chan->local : ~0U;
		memcpy(__entry->srv_name, chan ? chan->srv_name : "", MAX_SRV_NAME_LEN);
		__entry->buf_id = rxbuf ? rxbuf->buf_id : ~0ULL;
		__entry->shm_cnt = rxbuf ? rxbuf->shm_cnt : 0;
	),
	TP_printk("len_or_err=%d chan=%u srv_name=%s buf_id=0x%llx shm_cnt=%zu",
		__entry->len_or_err, __entry->chan, __entry->srv_name,
		__entry->buf_id, __entry->shm_cnt)
);

TRACE_EVENT(trusty_ipc_poll,
	TP_PROTO(struct tipc_chan *chan,
		unsigned int poll_mask),
	TP_ARGS(chan, poll_mask),
	TP_STRUCT__entry(
		__field(unsigned int, poll_mask)
		__field(u32, chan)
		__array(char, srv_name, MAX_SRV_NAME_LEN)
	),
	TP_fast_assign(
		__entry->poll_mask = poll_mask;
		__entry->chan = chan ? chan->local : ~0U;
		memcpy(__entry->srv_name, chan ? chan->srv_name : "", MAX_SRV_NAME_LEN);
	),
	TP_printk("poll_mask=%u chan=%u srv_name=%s",
		__entry->poll_mask, __entry->chan, __entry->srv_name)
);

/*
 * tracepoint when a message buffer is received from trusty
 * and is awaiting for its HAL consumer to read it
 */
TRACE_EVENT(trusty_ipc_rx,
	TP_PROTO(struct tipc_chan *chan, struct tipc_msg_buf *rxbuf),
	TP_ARGS(chan, rxbuf),
	TP_STRUCT__entry(
		__field(u32, chan)
		__array(char, srv_name, MAX_SRV_NAME_LEN)
		__field(u64, buf_id)
	),
	TP_fast_assign(
		__entry->chan = chan ? chan->local : ~0U;
		memcpy(__entry->srv_name, chan ? chan->srv_name : "", MAX_SRV_NAME_LEN);
		__entry->buf_id = rxbuf ? rxbuf->buf_id : ~0ULL;
	),
	TP_printk("chan=%u srv_name=%s buf_id=0x%llx", __entry->chan,
		__entry->srv_name, __entry->buf_id)
);
#endif /* _TRUSTY_IPC_TRACE_H */

#undef TRACE_INCLUDE_PATH
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_PATH .
#define TRACE_INCLUDE_FILE trusty-ipc-trace
#include <trace/define_trace.h>
