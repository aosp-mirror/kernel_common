/*
 * Driver model for vib triggers
 *
 * Copyright (C) 2013 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __LINUX_VIB_H_INCLUDED
#define __LINUX_VIB_H_INCLUDED

#include <linux/rwsem.h>

/* TODO: a simple usage example for vibrator trigger */

/* Vibrator driver to implement trigger: */
/* 1. Setup "struct vib_trigger_enabler", especially name/default_trigger/enable/trigger_data. */
/* 2. Register it by "vib_trigger_enabler_register()". */
/* 3. Unregister when module unloaded, by "vib_trigger_enabler_unregister()". */
/* 4. Release "struct vib_trigger_enabler" if necessary. */

/* Other drivers to use trigger: */
/* 1. Setup "struct vib_trigger", refer to led trigger. */
/* 2. Register it by "vib_trigger_register_simple()". */
/* 3. Control vibrator by "vib_trigger_event()". */
/* 4. Unregister when module unloaded, by "vib_trigger_unregister_simple()". */
/* 5. Release "struct vib_trigger", refer to led trigger. */

/*
 * Vibrator Triggers
 */
#ifdef CONFIG_VIB_TRIGGERS

struct vib_trigger_enabler {
	const char		*name;
	const char		*default_trigger;	/* Trigger to use */

	/* Protects the "trigger" field below */
	struct rw_semaphore	trigger_lock;

	void			(*enable)(struct vib_trigger_enabler *enabler, int value);
	struct vib_trigger	*trigger;
	void			*trigger_data;
};

struct vib_trigger {
	/* Trigger Properties */
	const char			*name;

	/* enabler under control by this trigger */
	rwlock_t			trig_container_lock;
	struct vib_trigger_enabler	*enabler;
};

/* Registration functions for simple triggers */
#define DEFINE_VIB_TRIGGER(x)		static struct vib_trigger *x;
#define DEFINE_VIB_TRIGGER_GLOBAL(x)	struct vib_trigger *x;
extern void vib_trigger_enabler_register(struct vib_trigger_enabler *enabler);
extern void vib_trigger_enabler_unregister(struct vib_trigger_enabler *enabler);
extern void vib_trigger_set(struct vib_trigger_enabler *enabler, struct vib_trigger *trigger);
extern void vib_trigger_set_default(struct vib_trigger_enabler *enabler);
extern void vib_trigger_register_simple(const char *name, struct vib_trigger **trigger);
extern void vib_trigger_unregister_simple(struct vib_trigger *trigger);
extern void vib_trigger_event(struct vib_trigger *trigger, int value);

#else

/* Triggers aren't active - null macros */
#define DEFINE_VIB_TRIGGER(x)
#define DEFINE_VIB_TRIGGER_GLOBAL(x)
#define vib_trigger_enabler_register(x) do {} while (0)
#define vib_trigger_enabler_unregister(x) do {} while (0)
#define vib_trigger_set(x, y) do {} while(0)
#define vib_trigger_set_default(x) do {} while(0)
#define vib_trigger_register_simple(x, y) do {} while(0)
#define vib_trigger_unregister_simple(x) do {} while(0)
#define vib_trigger_event(x, y) do {} while(0)

#endif

#endif		/* __LINUX_VIB_H_INCLUDED */
