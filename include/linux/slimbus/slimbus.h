/* Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _LINUX_SLIMBUS_H
#define _LINUX_SLIMBUS_H
#include <linux/module.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/mod_devicetable.h>


extern struct bus_type slimbus_type;

#define SLIM_CL_PER_SUPERFRAME		6144
#define SLIM_CL_PER_SUPERFRAME_DIV8	(SLIM_CL_PER_SUPERFRAME >> 3)
#define SLIM_MAX_CLK_GEAR		10
#define SLIM_MIN_CLK_GEAR		1
#define SLIM_CL_PER_SL			4
#define SLIM_SL_PER_SUPERFRAME		(SLIM_CL_PER_SUPERFRAME >> 2)
#define SLIM_FRM_SLOTS_PER_SUPERFRAME	16
#define SLIM_GDE_SLOTS_PER_SUPERFRAME	2

#define SLIM_MSG_MT_CORE			0x0
#define SLIM_MSG_MT_DEST_REFERRED_CLASS		0x1
#define SLIM_MSG_MT_DEST_REFERRED_USER		0x2
#define SLIM_MSG_MT_SRC_REFERRED_CLASS		0x5
#define SLIM_MSG_MT_SRC_REFERRED_USER		0x6

#define SLIM_MSG_MC_REPORT_PRESENT               0x1
#define SLIM_MSG_MC_ASSIGN_LOGICAL_ADDRESS       0x2
#define SLIM_MSG_MC_RESET_DEVICE                 0x4
#define SLIM_MSG_MC_CHANGE_LOGICAL_ADDRESS       0x8
#define SLIM_MSG_MC_CHANGE_ARBITRATION_PRIORITY  0x9
#define SLIM_MSG_MC_REQUEST_SELF_ANNOUNCEMENT    0xC
#define SLIM_MSG_MC_REPORT_ABSENT                0xF

#define SLIM_MSG_MC_CONNECT_SOURCE               0x10
#define SLIM_MSG_MC_CONNECT_SINK                 0x11
#define SLIM_MSG_MC_DISCONNECT_PORT              0x14
#define SLIM_MSG_MC_CHANGE_CONTENT               0x18

#define SLIM_MSG_MC_REQUEST_INFORMATION          0x20
#define SLIM_MSG_MC_REQUEST_CLEAR_INFORMATION    0x21
#define SLIM_MSG_MC_REPLY_INFORMATION            0x24
#define SLIM_MSG_MC_CLEAR_INFORMATION            0x28
#define SLIM_MSG_MC_REPORT_INFORMATION           0x29

#define SLIM_MSG_MC_BEGIN_RECONFIGURATION        0x40
#define SLIM_MSG_MC_NEXT_ACTIVE_FRAMER           0x44
#define SLIM_MSG_MC_NEXT_SUBFRAME_MODE           0x45
#define SLIM_MSG_MC_NEXT_CLOCK_GEAR              0x46
#define SLIM_MSG_MC_NEXT_ROOT_FREQUENCY          0x47
#define SLIM_MSG_MC_NEXT_PAUSE_CLOCK             0x4A
#define SLIM_MSG_MC_NEXT_RESET_BUS               0x4B
#define SLIM_MSG_MC_NEXT_SHUTDOWN_BUS            0x4C
#define SLIM_MSG_MC_NEXT_DEFINE_CHANNEL          0x50
#define SLIM_MSG_MC_NEXT_DEFINE_CONTENT          0x51
#define SLIM_MSG_MC_NEXT_ACTIVATE_CHANNEL        0x54
#define SLIM_MSG_MC_NEXT_DEACTIVATE_CHANNEL      0x55
#define SLIM_MSG_MC_NEXT_REMOVE_CHANNEL          0x58
#define SLIM_MSG_MC_RECONFIGURE_NOW              0x5F

#define SLIM_MSG_CLK_PAUSE_SEQ_FLG		(1U << 8)

#define SLIM_MSG_MC_REQUEST_VALUE                0x60
#define SLIM_MSG_MC_REQUEST_CHANGE_VALUE         0x61
#define SLIM_MSG_MC_REPLY_VALUE                  0x64
#define SLIM_MSG_MC_CHANGE_VALUE                 0x68

#define SLIM_CLK_FAST				0
#define SLIM_CLK_CONST_PHASE			1
#define SLIM_CLK_UNSPECIFIED			2

struct slim_controller;
struct slim_device;

#define SLIM_MSG_DEST_LOGICALADDR	0
#define SLIM_MSG_DEST_ENUMADDR		1
#define	SLIM_MSG_DEST_BROADCAST		3

struct slim_ele_access {
	u16			start_offset;
	u8			num_bytes;
	struct completion	*comp;
};

struct slim_framer {
	u8	e_addr[6];
	int	rootfreq;
	int	superfreq;
};
#define to_slim_framer(d) container_of(d, struct slim_framer, dev);

struct slim_addrt {
	bool	valid;
	u8	eaddr[6];
	u8	laddr;
};

struct slim_msg_txn {
	u8			rl;
	u8			mt;
	u16			mc;
	u8			dt;
	u16			ec;
	u8			len;
	u8			tid;
	u8			la;
	u8			*rbuf;
	const u8		*wbuf;
	struct completion	*comp;
};

enum slim_port_state {
	SLIM_P_FREE,
	SLIM_P_UNCFG,
	SLIM_P_CFG,
};

enum slim_port_req {
	SLIM_REQ_DEFAULT,
	SLIM_REQ_HALF_DUP,
	SLIM_REQ_MULTI_CH,
};

enum slim_port_cfg {
	SLIM_CFG_NONE,
	SLIM_CFG_PACKED,
	SLIM_CFG_ALIGN_MSB,
};

enum slim_port_flow {
	SLIM_SRC,
	SLIM_SINK,
};

enum slim_port_err {
	SLIM_P_INPROGRESS,
	SLIM_P_OVERFLOW,
	SLIM_P_UNDERFLOW,
	SLIM_P_DISCONNECT,
	SLIM_P_NOT_OWNED,
};

struct slim_port {
	enum slim_port_err	err;
	enum slim_port_state	state;
	enum slim_port_req	req;
	enum slim_port_cfg	cfg;
	enum slim_port_flow	flow;
	struct slim_ch		*ch;
	struct completion	*xcomp;
	struct slim_controller	*ctrl;
};

enum slim_ch_state {
	SLIM_CH_FREE,
	SLIM_CH_ALLOCATED,
	SLIM_CH_DEFINED,
	SLIM_CH_PENDING_ACTIVE,
	SLIM_CH_ACTIVE,
	SLIM_CH_SUSPENDED,
	SLIM_CH_PENDING_REMOVAL,
};

enum slim_ch_proto {
	SLIM_HARD_ISO,
	SLIM_AUTO_ISO,
	SLIM_PUSH,
	SLIM_PULL,
	SLIM_ASYNC_SMPLX,
	SLIM_ASYNC_HALF_DUP,
	SLIM_EXT_SMPLX,
	SLIM_EXT_HALF_DUP,
};

enum slim_ch_rate {
	SLIM_RATE_1HZ,
	SLIM_RATE_4000HZ,
	SLIM_RATE_11025HZ,
};

enum slim_ch_coeff {
	SLIM_COEFF_1,
	SLIM_COEFF_3,
};

enum slim_ch_control {
	SLIM_CH_ACTIVATE,
	SLIM_CH_SUSPEND,
	SLIM_CH_REMOVE,
};

enum slim_ch_dataf {
	SLIM_CH_DATAF_NOT_DEFINED = 0,
	SLIM_CH_DATAF_LPCM_AUDIO = 1,
	SLIM_CH_DATAF_IEC61937_COMP_AUDIO = 2,
	SLIM_CH_DATAF_PACKED_PDM_AUDIO = 3,
};

enum slim_ch_auxf {
	SLIM_CH_AUXF_NOT_APPLICABLE = 0,
	SLIM_CH_AUXF_ZCUV_TUNNEL_IEC60958 = 1,
	SLIM_CH_USER_DEFINED = 0xF,
};

struct slim_ch {
	enum slim_ch_proto	prot;
	enum slim_ch_rate	baser;
	enum slim_ch_dataf	dataf;
	enum slim_ch_auxf	auxf;
	u32			ratem;
	u32			sampleszbits;
};

struct slim_ich {
	struct slim_ch		prop;
	enum slim_ch_coeff	coeff;
	enum slim_ch_state	state;
	u16			nextgrp;
	u32			prrate;
	u32			offset;
	u32			newoff;
	u32			interval;
	u32			newintr;
	u32			seglen;
	u8			rootexp;
	u32			srch;
	u32			*sinkh;
	int			nsink;
	u8			chan;
	int			ref;
	int			def;
};

struct slim_sched {
	struct slim_ich	**chc3;
	int		num_cc3;
	struct slim_ich	**chc1;
	int		num_cc1;
	u32		subfrmcode;
	u32		usedslots;
	u32		msgsl;
	u32		pending_msgsl;
	struct mutex	m_reconf;
	u8		*slots;
};

enum slim_clk_state {
	SLIM_CLK_ACTIVE,
	SLIM_CLK_ENTERING_PAUSE,
	SLIM_CLK_PAUSE_FAILED,
	SLIM_CLK_PAUSED,
};
struct slim_controller {
	struct device		dev;
	unsigned int		nr;
	struct list_head	list;
	char			name[SLIMBUS_NAME_SIZE];
	int			clkgear;
	int			min_cg;
	int			max_cg;
	enum slim_clk_state	clk_state;
	struct completion	pause_comp;
	struct slim_framer	*a_framer;
	struct mutex		m_ctrl;
	struct slim_addrt	*addrt;
	u8			num_dev;
	struct list_head	devs;
	struct workqueue_struct *wq;
	struct slim_msg_txn	**txnt;
	u8			last_tid;
	struct slim_port	*ports;
	int			nports;
	struct slim_ich		*chans;
	int			nchans;
	u8			reserved;
	struct slim_sched	sched;
	struct completion	dev_released;
	int			(*xfer_msg)(struct slim_controller *ctrl,
				struct slim_msg_txn *txn);
	int			(*set_laddr)(struct slim_controller *ctrl,
				const u8 *ea, u8 elen, u8 laddr);
	int			(*allocbw)(struct slim_device *sb,
				int *subfrmc, int *clkgear);
	int			(*get_laddr)(struct slim_controller *ctrl,
				const u8 *ea, u8 elen, u8 *laddr);
	int			(*wakeup)(struct slim_controller *ctrl);
	int			(*alloc_port)(struct slim_controller *ctrl,
				u8 port);
	void			(*dealloc_port)(struct slim_controller *ctrl,
				u8 port);
	int			(*framer_handover)(struct slim_controller *ctrl,
				struct slim_framer *new_framer);
	int			(*port_xfer)(struct slim_controller *ctrl,
				u8 pn, phys_addr_t iobuf, u32 len,
				struct completion *comp);
	enum slim_port_err	(*port_xfer_status)(struct slim_controller *ctr,
				u8 pn, phys_addr_t *done_buf, u32 *done_len);
	int			(*xfer_user_msg)(struct slim_controller *ctrl,
				u8 la, u8 mt, u8 mc,
				struct slim_ele_access *msg, u8 *buf, u8 len);
};
#define to_slim_controller(d) container_of(d, struct slim_controller, dev)

struct slim_driver {
	int				(*probe)(struct slim_device *sldev);
	int				(*remove)(struct slim_device *sldev);
	void				(*shutdown)(struct slim_device *sldev);
	int				(*suspend)(struct slim_device *sldev,
					pm_message_t pmesg);
	int				(*resume)(struct slim_device *sldev);
	int				(*device_up)(struct slim_device *sldev);
	int				(*device_down)
						(struct slim_device *sldev);
	int				(*reset_device)
						(struct slim_device *sldev);

	struct device_driver		driver;
	const struct slim_device_id	*id_table;
};
#define to_slim_driver(d) container_of(d, struct slim_driver, driver)

struct slim_pending_ch {
	u8	chan;
	struct	list_head pending;
};

struct slim_device {
	struct device		dev;
	const char		*name;
	u8			e_addr[6];
	struct slim_driver	*driver;
	struct slim_controller	*ctrl;
	u8			laddr;
	bool			reported;
	struct list_head	mark_define;
	struct list_head	mark_suspend;
	struct list_head	mark_removal;
	bool			notified;
	struct list_head	dev_list;
	struct work_struct	wd;
	struct mutex		sldev_reconf;
	u32			pending_msgsl;
	u32			cur_msgsl;
};
#define to_slim_device(d) container_of(d, struct slim_device, dev)

struct slim_boardinfo {
	int			bus_num;
	struct slim_device	*slim_slave;
};


extern int slim_get_logical_addr(struct slim_device *sb, const u8 *e_addr,
					u8 e_len, u8 *laddr);



/*
 * Message API access routines.
 * @sb: client handle requesting elemental message reads, writes.
 * @msg: Input structure for start-offset, number of bytes to read.
 * @rbuf: data buffer to be filled with values read.
 * @len: data buffer size
 * @wbuf: data buffer containing value/information to be written
 * context: can sleep
 * Returns:
 * -EINVAL: Invalid parameters
 * -ETIMEDOUT: If controller could not complete the request. This may happen if
 *  the bus lines are not clocked, controller is not powered-on, slave with
 *  given address is not enumerated/responding.
 */
extern int slim_request_val_element(struct slim_device *sb,
					struct slim_ele_access *msg, u8 *buf,
					u8 len);
extern int slim_request_inf_element(struct slim_device *sb,
					struct slim_ele_access *msg, u8 *buf,
					u8 len);
extern int slim_change_val_element(struct slim_device *sb,
					struct slim_ele_access *msg,
					const u8 *buf, u8 len);
extern int slim_clear_inf_element(struct slim_device *sb,
					struct slim_ele_access *msg, u8 *buf,
					u8 len);
extern int slim_request_change_val_element(struct slim_device *sb,
					struct slim_ele_access *msg, u8 *rbuf,
					const u8 *wbuf, u8 len);
extern int slim_request_clear_inf_element(struct slim_device *sb,
					struct slim_ele_access *msg, u8 *rbuf,
					const u8 *wbuf, u8 len);

extern int slim_xfer_msg(struct slim_controller *ctrl,
			struct slim_device *sbdev, struct slim_ele_access *msg,
			u16 mc, u8 *rbuf, const u8 *wbuf, u8 len);

extern int slim_user_msg(struct slim_device *sb, u8 la, u8 mt, u8 mc,
				struct slim_ele_access *msg, u8 *buf, u8 len);


extern int slim_alloc_mgrports(struct slim_device *sb, enum slim_port_req req,
				int nports, u32 *rh, int hsz);

extern int slim_dealloc_mgrports(struct slim_device *sb, u32 *hdl, int hsz);

extern int slim_port_xfer(struct slim_device *sb, u32 ph, phys_addr_t iobuf,
				u32 len, struct completion *comp);

extern enum slim_port_err slim_port_get_xfer_status(struct slim_device *sb,
			u32 ph, phys_addr_t *done_buf, u32 *done_len);

extern int slim_connect_src(struct slim_device *sb, u32 srch, u16 chanh);

extern int slim_connect_sink(struct slim_device *sb, u32 *sinkh, int nsink,
				u16 chanh);
extern int slim_disconnect_ports(struct slim_device *sb, u32 *ph, int nph);

extern int slim_get_slaveport(u8 la, int idx, u32 *rh, enum slim_port_flow flw);



extern int slim_alloc_ch(struct slim_device *sb, u16 *chanh);

extern int slim_query_ch(struct slim_device *sb, u8 chan, u16 *chanh);
extern int slim_dealloc_ch(struct slim_device *sb, u16 chanh);


extern int slim_define_ch(struct slim_device *sb, struct slim_ch *prop,
				u16 *chanh, u8 nchan, bool grp, u16 *grph);

extern int slim_control_ch(struct slim_device *sb, u16 grpchanh,
				enum slim_ch_control chctrl, bool commit);

extern enum slim_ch_state slim_get_ch_state(struct slim_device *sb,
						u16 chanh);

extern int slim_reservemsg_bw(struct slim_device *sb, u32 bw_bps, bool commit);

extern int slim_reconfigure_now(struct slim_device *sb);

extern int slim_ctrl_clk_pause(struct slim_controller *ctrl, bool wakeup,
		u8 restart);

extern int slim_driver_register(struct slim_driver *drv);

extern void slim_driver_unregister(struct slim_driver *drv);

extern int slim_add_numbered_controller(struct slim_controller *ctrl);

extern int slim_del_controller(struct slim_controller *ctrl);

extern int slim_add_device(struct slim_controller *ctrl,
			struct slim_device *sbdev);

extern void slim_remove_device(struct slim_device *sbdev);

extern int slim_assign_laddr(struct slim_controller *ctrl, const u8 *e_addr,
				u8 e_len, u8 *laddr, bool valid);

void slim_report_absent(struct slim_device *sbdev);

void slim_framer_booted(struct slim_controller *ctrl);

extern void slim_msg_response(struct slim_controller *ctrl, u8 *reply, u8 tid,
				u8 len);

extern struct slim_controller *slim_busnum_to_ctrl(u32 busnum);

extern void slim_ctrl_add_boarddevs(struct slim_controller *ctrl);

#ifdef CONFIG_SLIMBUS
extern int slim_register_board_info(struct slim_boardinfo const *info,
					unsigned n);
#else
static inline int slim_register_board_info(struct slim_boardinfo const *info,
					unsigned n)
{
	return 0;
}
#endif

static inline void *slim_get_ctrldata(const struct slim_controller *dev)
{
	return dev_get_drvdata(&dev->dev);
}

static inline void slim_set_ctrldata(struct slim_controller *dev, void *data)
{
	dev_set_drvdata(&dev->dev, data);
}

static inline void *slim_get_devicedata(const struct slim_device *dev)
{
	return dev_get_drvdata(&dev->dev);
}

static inline void slim_set_clientdata(struct slim_device *dev, void *data)
{
	dev_set_drvdata(&dev->dev, data);
}
#endif 
