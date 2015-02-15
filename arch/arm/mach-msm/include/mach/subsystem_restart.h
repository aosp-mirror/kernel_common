/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __SUBSYS_RESTART_H
#define __SUBSYS_RESTART_H

#include <linux/spinlock.h>
#include <linux/interrupt.h>

#define SUBSYS_NAME_MAX_LENGTH 40

struct subsys_device;

enum {
	RESET_SOC = 0,
	RESET_SUBSYS_COUPLED,
	RESET_LEVEL_MAX
};

#if defined(CONFIG_HTC_FEATURES_SSR)
enum {
	DISABLE_RAMDUMP = 0,
	ENABLE_RAMDUMP,
};
#endif

struct device;
struct module;

/**
 * struct subsys_desc - subsystem descriptor
 * @name: name of subsystem
 * @depends_on: subsystem this subsystem depends on to operate
 * @dev: parent device
 * @owner: module the descriptor belongs to
 * @start: Start a subsystem
 * @stop: Stop a subsystem
 * @shutdown: Stop a subsystem
 * @powerup: Start a subsystem
 * @crash_shutdown: Shutdown a subsystem when the system crashes (can't sleep)
 * @ramdump: Collect a ramdump of the subsystem
 * @is_not_loadable: Indicate if subsystem firmware is not loadable via pil
 * framework
 */
struct subsys_desc {
	const char *name;
	const char *depends_on;
	struct device *dev;
	struct module *owner;

	int (*start)(const struct subsys_desc *desc);
	void (*stop)(const struct subsys_desc *desc);

	int (*shutdown)(const struct subsys_desc *desc);
	int (*powerup)(const struct subsys_desc *desc);
	void (*crash_shutdown)(const struct subsys_desc *desc);
	int (*ramdump)(int, const struct subsys_desc *desc);
	irqreturn_t (*err_fatal_handler) (int irq, void *dev_id);
	irqreturn_t (*stop_ack_handler) (int irq, void *dev_id);
	irqreturn_t (*wdog_bite_handler) (int irq, void *dev_id);
	int is_not_loadable;
	unsigned int err_fatal_irq;
	unsigned int err_ready_irq;
	unsigned int stop_ack_irq;
	unsigned int wdog_bite_irq;
	int force_stop_gpio;
};

#if defined(CONFIG_MSM_SUBSYSTEM_RESTART)

#if defined(CONFIG_HTC_DEBUG_SSR)
void subsys_set_restart_reason(struct subsys_device *dev, const char *reason);
#endif /* CONFIG_HTC_DEBUG_SSR  */

#if defined(CONFIG_HTC_FEATURES_SSR)
extern void subsys_set_enable_ramdump(struct subsys_device *dev, int enable);
extern void subsys_set_restart_level(struct subsys_device *dev, int level);
#endif

extern int subsys_get_restart_level(struct subsys_device *dev);
extern int subsystem_restart_dev(struct subsys_device *dev);
extern int subsystem_restart(const char *name);
extern int subsystem_crashed(const char *name);

extern void *subsystem_get(const char *name);
extern void subsystem_put(void *subsystem);

extern struct subsys_device *subsys_register(struct subsys_desc *desc);
extern void subsys_unregister(struct subsys_device *dev);

extern void subsys_default_online(struct subsys_device *dev);
extern void subsys_set_crash_status(struct subsys_device *dev, bool crashed);
extern bool subsys_get_crash_status(struct subsys_device *dev);

#else

#if defined(CONFIG_HTC_DEBUG_SSR)
static inline void subsys_set_restart_reason(struct subsys_device *dev, const char *reason)
{
	return;
}
#endif /* CONFIG_HTC_DEBUG_SSR */

#if defined(CONFIG_HTC_FEATURES_SSR)
static inline void subsys_set_enable_ramdump(struct subsys_device *dev, int enable)
{
	return 0;
}

static inline void subsys_set_restart_level(struct subsys_device *dev, int level)
{
	return 0;
}
#endif

static inline int subsys_get_restart_level(struct subsys_device *dev)
{
	return 0;
}

static inline int subsystem_restart_dev(struct subsys_device *dev)
{
	return 0;
}

static inline int subsystem_restart(const char *name)
{
	return 0;
}

static inline int subsystem_crashed(const char *name)
{
	return 0;
}

static inline void *subsystem_get(const char *name)
{
	return NULL;
}

static inline void subsystem_put(void *subsystem) { }

static inline
struct subsys_device *subsys_register(struct subsys_desc *desc)
{
	return NULL;
}

static inline void subsys_unregister(struct subsys_device *dev) { }

static inline void subsys_default_online(struct subsys_device *dev) { }
static inline
void subsys_set_crash_status(struct subsys_device *dev, bool crashed) { }
static inline bool subsys_get_crash_status(struct subsys_device *dev)
{
	return false;
}

#endif /* CONFIG_MSM_SUBSYSTEM_RESTART */

#endif
