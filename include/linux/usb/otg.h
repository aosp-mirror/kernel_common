
#ifndef __LINUX_USB_OTG_H
#define __LINUX_USB_OTG_H

#include <linux/notifier.h>

enum usb_otg_state {
	OTG_STATE_UNDEFINED = 0,

	
	OTG_STATE_B_IDLE,
	OTG_STATE_B_SRP_INIT,
	OTG_STATE_B_PERIPHERAL,

	
	OTG_STATE_B_WAIT_ACON,
	OTG_STATE_B_HOST,

	
	OTG_STATE_A_IDLE,
	OTG_STATE_A_WAIT_VRISE,
	OTG_STATE_A_WAIT_BCON,
	OTG_STATE_A_HOST,
	OTG_STATE_A_SUSPEND,
	OTG_STATE_A_PERIPHERAL,
	OTG_STATE_A_WAIT_VFALL,
	OTG_STATE_A_VBUS_ERR,
};

enum usb_otg_event {
	OTG_EVENT_DEV_CONN_TMOUT,
	OTG_EVENT_NO_RESP_FOR_HNP_ENABLE,
	OTG_EVENT_HUB_NOT_SUPPORTED,
	OTG_EVENT_DEV_NOT_SUPPORTED,
	OTG_EVENT_HNP_FAILED,
	OTG_EVENT_NO_RESP_FOR_SRP,
	OTG_EVENT_INSUFFICIENT_POWER,
};

enum usb_phy_events {
	USB_EVENT_NONE,         
	USB_EVENT_VBUS,         
	USB_EVENT_ID,           
	USB_EVENT_CHARGER,      
	USB_EVENT_ENUMERATED,   
};

struct usb_phy;

struct usb_phy_io_ops {
	int (*read)(struct usb_phy *x, u32 reg);
	int (*write)(struct usb_phy *x, u32 val, u32 reg);
};

struct usb_otg {
	u8			default_a;

	struct usb_phy		*phy;
	struct usb_bus		*host;
	struct usb_gadget	*gadget;

	
	int	(*set_host)(struct usb_otg *otg, struct usb_bus *host);

	
	int	(*set_peripheral)(struct usb_otg *otg,
					struct usb_gadget *gadget);

	
	int	(*set_vbus)(struct usb_otg *otg, bool enabled);

	
	int	(*start_srp)(struct usb_otg *otg);

	
	int	(*start_hnp)(struct usb_otg *otg);

	
	int	(*send_event)(struct usb_otg *otg,
			enum usb_otg_event event);

};

struct usb_phy {
	struct device		*dev;
	const char		*label;
	unsigned int		 flags;

	enum usb_otg_state	state;
	enum usb_phy_events	last_event;

	struct usb_otg		*otg;

	struct device		*io_dev;
	struct usb_phy_io_ops	*io_ops;
	void __iomem		*io_priv;

	
	struct atomic_notifier_head	notifier;

	
	u16			port_status;
	u16			port_change;

	
	int	(*init)(struct usb_phy *x);
	void	(*shutdown)(struct usb_phy *x);

	
	int	(*set_power)(struct usb_phy *x,
				unsigned mA);

	
	int	(*set_suspend)(struct usb_phy *x,
				int suspend);

	
	void	(*notify_usb_attached)(struct usb_phy *x);
	
	void	(*notify_usb_disabled)(void);

	
	int	(*set_phy_autosuspend)(struct usb_phy *x,
					int enable_autosuspend);
};


extern int usb_set_transceiver(struct usb_phy *);

#if defined(CONFIG_NOP_USB_XCEIV) || (defined(CONFIG_NOP_USB_XCEIV_MODULE) && defined(MODULE))
extern void usb_nop_xceiv_register(void);
extern void usb_nop_xceiv_unregister(void);
#else
static inline void usb_nop_xceiv_register(void)
{
}

static inline void usb_nop_xceiv_unregister(void)
{
}
#endif

static inline int usb_phy_io_read(struct usb_phy *x, u32 reg)
{
	if (x->io_ops && x->io_ops->read)
		return x->io_ops->read(x, reg);

	return -EINVAL;
}

static inline int usb_phy_io_write(struct usb_phy *x, u32 val, u32 reg)
{
	if (x->io_ops && x->io_ops->write)
		return x->io_ops->write(x, val, reg);

	return -EINVAL;
}

static inline int
usb_phy_init(struct usb_phy *x)
{
	if (x->init)
		return x->init(x);

	return 0;
}

static inline void
usb_phy_shutdown(struct usb_phy *x)
{
	if (x->shutdown)
		x->shutdown(x);
}

extern int otg_send_event(enum usb_otg_event event);

#ifdef CONFIG_USB_OTG_UTILS
extern struct usb_phy *usb_get_transceiver(void);
extern void usb_put_transceiver(struct usb_phy *);
extern const char *otg_state_string(enum usb_otg_state state);
#else
static inline struct usb_phy *usb_get_transceiver(void)
{
	return NULL;
}

static inline void usb_put_transceiver(struct usb_phy *x)
{
}

static inline const char *otg_state_string(enum usb_otg_state state)
{
	return NULL;
}
#endif

static inline int
otg_start_hnp(struct usb_otg *otg)
{
	if (otg && otg->start_hnp)
		return otg->start_hnp(otg);

	return -ENOTSUPP;
}

static inline int
otg_set_vbus(struct usb_otg *otg, bool enabled)
{
	if (otg && otg->set_vbus)
		return otg->set_vbus(otg, enabled);

	return -ENOTSUPP;
}

static inline int
otg_set_host(struct usb_otg *otg, struct usb_bus *host)
{
	if (otg && otg->set_host)
		return otg->set_host(otg, host);

	return -ENOTSUPP;
}


static inline int
otg_set_peripheral(struct usb_otg *otg, struct usb_gadget *periph)
{
	if (otg && otg->set_peripheral)
		return otg->set_peripheral(otg, periph);

	return -ENOTSUPP;
}

static inline int
usb_phy_set_power(struct usb_phy *x, unsigned mA)
{
	if (x && x->set_power)
		return x->set_power(x, mA);
	return 0;
}

static inline int
usb_phy_set_suspend(struct usb_phy *x, int suspend)
{
	if (x->set_suspend != NULL)
		return x->set_suspend(x, suspend);
	else
		return 0;
}

static inline int
usb_phy_set_autosuspend(struct usb_phy *x, int enable_autosuspend)
{
	if (x && x->set_phy_autosuspend != NULL)
		return x->set_phy_autosuspend(x, enable_autosuspend);
	else
		return 0;
}

static inline int
otg_start_srp(struct usb_otg *otg)
{
	if (otg && otg->start_srp)
		return otg->start_srp(otg);

	return -ENOTSUPP;
}

static inline int
usb_register_notifier(struct usb_phy *x, struct notifier_block *nb)
{
	return atomic_notifier_chain_register(&x->notifier, nb);
}

static inline void
usb_unregister_notifier(struct usb_phy *x, struct notifier_block *nb)
{
	atomic_notifier_chain_unregister(&x->notifier, nb);
}

extern int usb_bus_start_enum(struct usb_bus *bus, unsigned port_num);

#endif 
