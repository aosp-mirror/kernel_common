/*
 * Vibrator Triggers Core
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/rwsem.h>
#include <linux/slab.h>

#include <linux/vibtrig.h>

/*
 * Singleton of vib_trigger, and a lock to protect this object
 */
static DECLARE_RWSEM(triggers_container_lock);
static struct vib_trigger* trigger_singleton = NULL;

/*
 * Singleton of vib_trigger_enabler, and a lock to protect this object
 */

static DECLARE_RWSEM(enabler_container_lock);
static struct vib_trigger_enabler* enabler_singleton = NULL;

/* Simple Vibrator Trigger Interface */

void vib_trigger_enabler_register(struct vib_trigger_enabler *enabler)
{
	int err;

	init_rwsem(&enabler->trigger_lock);

	/* the default place to assign enabler_singleton */
	down_write(&enabler_container_lock);
	if (enabler_singleton != NULL) {
		err = -EEXIST;
	} else {
		enabler_singleton = enabler;
		vib_trigger_set_default(enabler);
		err = 0;
	}
	up_write(&enabler_container_lock);

	printk(KERN_DEBUG
			"Registered vibrator trigger enabler: %s\n",
			enabler->name);
}
EXPORT_SYMBOL_GPL(vib_trigger_enabler_register);

void vib_trigger_enabler_unregister(struct vib_trigger_enabler *enabler)
{
	down_write(&enabler->trigger_lock);
	if (enabler->trigger)
		vib_trigger_set(enabler, NULL);
	up_write(&enabler->trigger_lock);

	/* Stop any motion */
	enabler->enable(enabler, 0);

	down_write(&enabler_container_lock);
	enabler_singleton = NULL;
	up_write(&enabler_container_lock);
}
EXPORT_SYMBOL_GPL(vib_trigger_enabler_unregister);

/* Caller must ensure enabler->trigger_lock held */
void vib_trigger_set(struct vib_trigger_enabler *enabler, struct vib_trigger *trigger)
{
	unsigned long flags;

	/* Remove any existing trigger */
	if (enabler->trigger) {
		write_lock_irqsave(&enabler->trigger->trig_container_lock, flags);
		enabler->trigger->enabler = NULL;
		write_unlock_irqrestore(&enabler->trigger->trig_container_lock, flags);
		enabler->trigger = NULL;
		enabler->enable(enabler, 0);
	}
	/* Bind trigger/enabler */
	if (trigger) {
		write_lock_irqsave(&trigger->trig_container_lock, flags);
		trigger->enabler = enabler;
		write_unlock_irqrestore(&trigger->trig_container_lock, flags);
		enabler->trigger = trigger;
	}
}
EXPORT_SYMBOL_GPL(vib_trigger_set);

void vib_trigger_set_default(struct vib_trigger_enabler *enabler)
{
	if (!enabler->default_trigger)
		return;

	down_read(&triggers_container_lock);
	down_write(&enabler->trigger_lock);
	if (trigger_singleton != NULL)
		if (!strcmp(enabler->default_trigger, trigger_singleton->name)) {
			/* name matched */
			vib_trigger_set(enabler, trigger_singleton);
		}
	up_write(&enabler->trigger_lock);
	up_read(&triggers_container_lock);
}
EXPORT_SYMBOL_GPL(vib_trigger_set_default);

void vib_trigger_event(struct vib_trigger *trigger, int value)
{
	if (!trigger)
		return;

	read_lock(&trigger->trig_container_lock);
	if (trigger->enabler)
		trigger->enabler->enable(trigger->enabler, value);
	read_unlock(&trigger->trig_container_lock);
}
EXPORT_SYMBOL_GPL(vib_trigger_event);

void vib_trigger_register_simple(const char *name, struct vib_trigger **tp)
{
	struct vib_trigger *trigger;
	int err;

	trigger = kzalloc(sizeof(struct vib_trigger), GFP_KERNEL);

	if (trigger) {
		trigger->name = name;
		rwlock_init(&trigger->trig_container_lock);
		trigger->enabler = NULL;

		/* the only place to assign trigger_singleton */
		down_write(&triggers_container_lock);
		if (trigger_singleton != NULL) {
			err = -EEXIST;
		} else {
			trigger_singleton = trigger;
			err = 0;
		}
		up_write(&triggers_container_lock);

		if (err == 0) {
			/* Register with any vibrator trigger enabler that has this as a default trigger */
			down_read(&enabler_container_lock);
			if (enabler_singleton != NULL) {
				down_write(&enabler_singleton->trigger_lock);
				if (!enabler_singleton->trigger && enabler_singleton->default_trigger &&
					    !strcmp(enabler_singleton->default_trigger, trigger->name))
					vib_trigger_set(enabler_singleton, trigger);
				up_write(&enabler_singleton->trigger_lock);
			}
			up_read(&enabler_container_lock);
		}

		if (err < 0) {
			kfree(trigger);
			trigger = NULL;
			printk(KERN_WARNING
				"Vibrator trigger %s failed to register"
				" (%d)\n", name, err);
		}
	} else
		printk(KERN_WARNING
			"Vibrator trigger %s failed to register"
			" (no memory)\n", name);

	*tp = trigger;
}
EXPORT_SYMBOL_GPL(vib_trigger_register_simple);

void vib_trigger_unregister_simple(struct vib_trigger *trigger)
{
	if (trigger)
	{
		struct vib_trigger_enabler* enabler;

		/* Remove from the Singleton of vib triggers */
		down_write(&triggers_container_lock);
		trigger_singleton = NULL;
		up_write(&triggers_container_lock);

		/* Remove anyone actively using this trigger */
		enabler = trigger->enabler;
		if (enabler) {
			down_write(&enabler->trigger_lock);
			if (enabler->trigger == trigger)
				enabler->trigger = NULL;
			up_write(&enabler->trigger_lock);
		}
	}
	kfree(trigger);
}
EXPORT_SYMBOL_GPL(vib_trigger_unregister_simple);

MODULE_AUTHOR("HTC");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Vibrator Triggers Core");
