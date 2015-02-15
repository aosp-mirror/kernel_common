#ifndef KGSL_HTC_H
#define KGSL_HTC_H

#include <linux/msm_kgsl.h>

struct kgsl_device;
struct idr;

struct kgsl_driver_htc_priv {
	struct work_struct work;
	unsigned long next_jiffies;
};

int kgsl_driver_htc_init(struct kgsl_driver_htc_priv *priv);

int kgsl_device_htc_init(struct kgsl_device *device);

void kgsl_dump_contextpid_locked(struct idr *context_idr);

void adreno_fault_panic(struct kgsl_device *device, unsigned int pid, int fault);

#endif
