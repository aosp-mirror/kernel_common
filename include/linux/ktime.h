/*
 *  include/linux/ktime.h
 *
 *  ktime_t - nanosecond-resolution time format.
 *
 *   Copyright(C) 2005, Thomas Gleixner <tglx@linutronix.de>
 *   Copyright(C) 2005, Red Hat, Inc., Ingo Molnar
 *
 *  data type definitions, declarations, prototypes and macros.
 *
 *  Started by: Thomas Gleixner and Ingo Molnar
 *
 *  Credits:
 *
 *  	Roman Zippel provided the ideas and primary code snippets of
 *  	the ktime_t union and further simplifications of the original
 *  	code.
 *
 *  For licencing details see kernel-base/COPYING
 */
#ifndef _LINUX_KTIME_H
#define _LINUX_KTIME_H

#include <linux/time.h>
#include <linux/jiffies.h>

union ktime {
	s64	tv64;
#if BITS_PER_LONG != 64 && !defined(CONFIG_KTIME_SCALAR)
	struct {
# ifdef __BIG_ENDIAN
	s32	sec, nsec;
# else
	s32	nsec, sec;
# endif
	} tv;
#endif
};

typedef union ktime ktime_t;		

#define KTIME_MAX			((s64)~((u64)1 << 63))
#if (BITS_PER_LONG == 64)
# define KTIME_SEC_MAX			(KTIME_MAX / NSEC_PER_SEC)
#else
# define KTIME_SEC_MAX			LONG_MAX
#endif


#if (BITS_PER_LONG == 64) || defined(CONFIG_KTIME_SCALAR)

static inline ktime_t ktime_set(const long secs, const unsigned long nsecs)
{
#if (BITS_PER_LONG == 64)
	if (unlikely(secs >= KTIME_SEC_MAX))
		return (ktime_t){ .tv64 = KTIME_MAX };
#endif
	return (ktime_t) { .tv64 = (s64)secs * NSEC_PER_SEC + (s64)nsecs };
}

#define ktime_sub(lhs, rhs) \
		({ (ktime_t){ .tv64 = (lhs).tv64 - (rhs).tv64 }; })

#define ktime_add(lhs, rhs) \
		({ (ktime_t){ .tv64 = (lhs).tv64 + (rhs).tv64 }; })

#define ktime_add_ns(kt, nsval) \
		({ (ktime_t){ .tv64 = (kt).tv64 + (nsval) }; })

#define ktime_sub_ns(kt, nsval) \
		({ (ktime_t){ .tv64 = (kt).tv64 - (nsval) }; })

static inline ktime_t timespec_to_ktime(struct timespec ts)
{
	return ktime_set(ts.tv_sec, ts.tv_nsec);
}

static inline ktime_t timeval_to_ktime(struct timeval tv)
{
	return ktime_set(tv.tv_sec, tv.tv_usec * NSEC_PER_USEC);
}

#define ktime_to_timespec(kt)		ns_to_timespec((kt).tv64)

#define ktime_to_timeval(kt)		ns_to_timeval((kt).tv64)

#define ktime_to_ns(kt)			((kt).tv64)

#else	


static inline ktime_t ktime_set(const long secs, const unsigned long nsecs)
{
	return (ktime_t) { .tv = { .sec = secs, .nsec = nsecs } };
}

static inline ktime_t ktime_sub(const ktime_t lhs, const ktime_t rhs)
{
	ktime_t res;

	res.tv64 = lhs.tv64 - rhs.tv64;
	if (res.tv.nsec < 0)
		res.tv.nsec += NSEC_PER_SEC;

	return res;
}

static inline ktime_t ktime_add(const ktime_t add1, const ktime_t add2)
{
	ktime_t res;

	res.tv64 = add1.tv64 + add2.tv64;
	if (res.tv.nsec >= NSEC_PER_SEC)
		res.tv64 += (u32)-NSEC_PER_SEC;

	return res;
}

extern ktime_t ktime_add_ns(const ktime_t kt, u64 nsec);

extern ktime_t ktime_sub_ns(const ktime_t kt, u64 nsec);

static inline ktime_t timespec_to_ktime(const struct timespec ts)
{
	return (ktime_t) { .tv = { .sec = (s32)ts.tv_sec,
			   	   .nsec = (s32)ts.tv_nsec } };
}

static inline ktime_t timeval_to_ktime(const struct timeval tv)
{
	return (ktime_t) { .tv = { .sec = (s32)tv.tv_sec,
				   .nsec = (s32)tv.tv_usec * 1000 } };
}

static inline struct timespec ktime_to_timespec(const ktime_t kt)
{
	return (struct timespec) { .tv_sec = (time_t) kt.tv.sec,
				   .tv_nsec = (long) kt.tv.nsec };
}

static inline struct timeval ktime_to_timeval(const ktime_t kt)
{
	return (struct timeval) {
		.tv_sec = (time_t) kt.tv.sec,
		.tv_usec = (suseconds_t) (kt.tv.nsec / NSEC_PER_USEC) };
}

static inline s64 ktime_to_ns(const ktime_t kt)
{
	return (s64) kt.tv.sec * NSEC_PER_SEC + kt.tv.nsec;
}

#endif	

static inline int ktime_equal(const ktime_t cmp1, const ktime_t cmp2)
{
	return cmp1.tv64 == cmp2.tv64;
}

static inline int ktime_compare(const ktime_t cmp1, const ktime_t cmp2)
{
	if (cmp1.tv64 < cmp2.tv64)
		return -1;
	if (cmp1.tv64 > cmp2.tv64)
		return 1;
	return 0;
}

static inline s64 ktime_to_us(const ktime_t kt)
{
	struct timeval tv = ktime_to_timeval(kt);
	return (s64) tv.tv_sec * USEC_PER_SEC + tv.tv_usec;
}

static inline s64 ktime_to_ms(const ktime_t kt)
{
	struct timeval tv = ktime_to_timeval(kt);
	return (s64) tv.tv_sec * MSEC_PER_SEC + tv.tv_usec / USEC_PER_MSEC;
}

static inline s64 ktime_us_delta(const ktime_t later, const ktime_t earlier)
{
       return ktime_to_us(ktime_sub(later, earlier));
}

static inline ktime_t ktime_add_us(const ktime_t kt, const u64 usec)
{
	return ktime_add_ns(kt, usec * 1000);
}

static inline ktime_t ktime_sub_us(const ktime_t kt, const u64 usec)
{
	return ktime_sub_ns(kt, usec * 1000);
}

extern ktime_t ktime_add_safe(const ktime_t lhs, const ktime_t rhs);

#define LOW_RES_NSEC		TICK_NSEC
#define KTIME_LOW_RES		(ktime_t){ .tv64 = LOW_RES_NSEC }

extern void ktime_get_ts(struct timespec *ts);

#define ktime_get_real_ts(ts)	getnstimeofday(ts)

static inline ktime_t ns_to_ktime(u64 ns)
{
	static const ktime_t ktime_zero = { .tv64 = 0 };
	return ktime_add_ns(ktime_zero, ns);
}

#endif
