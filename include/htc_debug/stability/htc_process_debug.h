
#ifdef CONFIG_HTC_PROCESS_DEBUG
extern void send_signal_debug_dump(int sig, struct task_struct *t);
extern void do_group_exit_debug_dump(int exit_code);
#else
static inline void send_signal_debug_dump(int sig, struct task_struct *t) {};
static inline void do_group_exit_debug_dump(int exit_code) {};
#endif
