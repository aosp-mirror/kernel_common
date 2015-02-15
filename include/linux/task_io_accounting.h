
struct task_io_accounting {
#ifdef CONFIG_TASK_XACCT
	
	u64 rchar;
	/*  bytes written */
	u64 wchar;
	
	u64 syscr;
	
	u64 syscw;
#endif 

#ifdef CONFIG_TASK_IO_ACCOUNTING
	u64 read_bytes;

	/*
	 * The number of bytes which this task has caused, or shall cause to be
	 * written to disk.
	 */
	u64 write_bytes;

	u64 cancelled_write_bytes;
#endif 

	
	u64 acc_bytes[3];
	unsigned long last_jiffies[3];
};
