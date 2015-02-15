/*

SiI8558 / SiI8240 Linux Driver

Copyright (C) 2013 Silicon Image, Inc.

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License as
published by the Free Software Foundation version 2.
This program is distributed AS-IS WITHOUT ANY WARRANTY of any
kind, whether express or implied; INCLUDING without the implied warranty
of MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE or NON-INFRINGEMENT.  See
the GNU General Public License for more details at http://www.gnu.org/licenses/gpl-2.0.html.

*/

#if !defined(MHL_LINUXDRV_IOCTL_H)
#define MHL_LINUXDRV_IOCTL_H

#include <linux/ioctl.h>


#ifdef __cplusplus
extern "C" {
#endif  /* _defined (__cplusplus) */


/**
 * struct mhl_tx_event_packet - Contains information about a driver event
 * @event:			Event code.
 * @event_param:	Event code specific additional event parameter
 *
 * This structure is used with the SII_MHL_GET_MHL_TX_EVENT IOCTL to retrieve
 * event information from the driver.
 */
struct mhl_tx_event_packet {
	__u32	event;
	__u32	event_param;
};


/**
 * struct mhl_tx_read_dev_cap - Used to retrieve a Device Capabilities register
 * @reg_num:	Input specifying Dev Cap register to read
 * @reg_value:	On return contains the value of Dev Cap register regNum
 *
 * This structure is used with the SII_MHL_GET_DEV_CAP_VALUE IOCTL to retrieve
 * the value of one of the Device Capability registers of the device attached
 * to the MHL transmitter.
 */
struct mhl_tx_read_dev_cap {
	__u8	reg_num;
	__u8	reg_value;
};


/**
 * struct mhl_tx_scratch_pad_access - Used to read or write Scratch Pad registers
 * @offset:	Offset within the Scratch Pad register space to begin writing to,
 * 			or reading from
 * @length:	Number of Scratch Pad registers to write or read
 * @data:	Buffer containing write or read data
 *
 * This structure is used with the SII_MHL_WRITE_SCRATCH_PAD and SII_MHL_READ_SCRATCH_PAD
 * IOCTLs to write or read respectively the scratch pad registers of the device
 * attached to the MHL transmitter.
  */
#define ADOPTER_ID_SIZE					2
#define MAX_SCRATCH_PAD_TRANSFER_SIZE	16
#define SCRATCH_PAD_SIZE				64
struct mhl_tx_scratch_pad_access {
	__u8	offset;
	__u8 	length;
	__u8	data[MAX_SCRATCH_PAD_TRANSFER_SIZE];
};



/*
 * Event codes returned by IOCTL SII_MHL_GET_MHL_TX_EVENT
 *
 */
/* No event worth reporting */
#define	MHL_TX_EVENT_NONE			0x00

/* MHL connection has been lost */
#define	MHL_TX_EVENT_DISCONNECTION	0x01

/* MHL connection has been established */
#define	MHL_TX_EVENT_CONNECTION		0x02

/* Received an RCP key code */
#define	MHL_TX_EVENT_RCP_RECEIVED	0x04

/* Received an RCPK message */
#define	MHL_TX_EVENT_RCPK_RECEIVED	0x05

/* Received an RCPE message */
#define	MHL_TX_EVENT_RCPE_RECEIVED	0x06

/* Received an UTF-8 key code */
#define	MHL_TX_EVENT_UCP_RECEIVED	0x07

/* Received an UCPK message */
#define	MHL_TX_EVENT_UCPK_RECEIVED	0x08

/* Received an UCPE message */
#define	MHL_TX_EVENT_UCPE_RECEIVED	0x09

/* Scratch Pad Data received */
#define MHL_TX_EVENT_SPAD_RECEIVED	0x0A

/* Peer's power capability has changed */
#define	MHL_TX_EVENT_POW_BIT_CHG	0x0B

/* Received a Request Action Protocol (RAP) message */
#define MHL_TX_EVENT_RAP_RECEIVED	0x0C


/*
 * Error status codes for RCPE messages
 */
/* No error.  (Not allowed in RCPE messages) */
#define	MHL_RCPE_STATUS_NO_ERROR				0x00
/* Unsupported/unrecognized key code */
#define	MHL_RCPE_STATUS_INEEFECTIVE_KEY_CODE	0x01
/* Responder busy.  Initiator may retry message */
#define	MHL_RCPE_STATUS_BUSY					0x02

/* Define type field for Silicon Image MHL I/O control codes */
#define IOC_SII_MHL_TYPE ('S')


/*
 * IOCTL to poll the driver for events
 *
 * An application needs to send this IOCTL periodically to poll the driver
 * for MHL connection related events.  Events reported by this IOCTL fall
 * generally into two categories, MHL physical link related (connection /
 * disconnection), or Remote Control Protocol (RCP).
 *
 * With regard to RCP events the MHL specification states that RCP events
 * that are not responded to within the TRCP_WAIT period (1000ms. max) may
 * be regarded as failed transmissions by the sender.  Therefore applications
 * should call this function at intervals less than TRCP_WAIT to avoid RCP
 * message timeouts.
 *
 * The parameter passed with this IOCTL is a pointer to an mhl_tx_event_packet
 * structure.  Upon return the structure will contain the information about
 * the next event (if any) detected by the driver.
 */
#define SII_MHL_GET_MHL_TX_EVENT \
    _IOR(IOC_SII_MHL_TYPE, 0x01, struct mhl_tx_event_packet *)


/*
 * IOCTL to send a remote control code
 *
 * An application uses this IOCTL code to send an MHL defined remote control
 * code to the downstream display device.
 *
 * The parameter passed with this IOCTL is the remote control key code to be
 * sent to the downstream display device.
 */
#define SII_MHL_RCP_SEND \
    _IOW(IOC_SII_MHL_TYPE, 0x02, __u8)


/*
 * IOCTL to send a remote control code acknowledgment
 *
 * An application uses this IOCTL code to send a positive acknowledgment for
 * an RCP code received from the downstream display device and reported to the
 * application via the SII_MHL_GET_MHL_TX_EVENT IOCTL.
 *
 * The parameter passed with this IOCTL is the remote control key code that
 * is being acknowledged.
 */
#define SII_MHL_RCP_SEND_ACK \
    _IOW(IOC_SII_MHL_TYPE, 0x03, __u8)


/*
 * IOCTL to send a remote control code error acknowledgment
 *
 * An application uses this IOCTL code to send a negative acknowledgment for
 * an RCP code received from the downstream display device and reported to the
 * application via the SII_MHL_GET_MHL_TX_EVENT IOCTL.
 *
 * The parameter passed with this IOCTL is the remote control key code to be
 * negatively acknowledged.
 */
#define SII_MHL_RCP_SEND_ERR_ACK \
    _IOW(IOC_SII_MHL_TYPE, 0x04, __u8)


/*
 * IOCTL to retrieve status of MHL connection
 *
 * An application can use this IOCTL to determine the current state of the
 * transmitter's MHL port.
 *
 * The parameter passed with this IOCTL is a pointer to an mhl_tx_event_packet
 * structure.  Upon return the event field of this structure will contain the
 * most recent event code pertaining to the status of the MHL connection.
 */
#define SII_MHL_GET_MHL_CONNECTION_STATUS \
    _IOR(IOC_SII_MHL_TYPE, 0x05, struct mhl_tx_event_packet *)


/*
 * IOCTL to retrieve the value of one of the Device Capability registers
 *
 * An application can use this IOCTL to read the value of one of the Device
 * Capability registers from the device attached to the MHL transmitter.
 *
 * The parameter passed with this IOCTL is a pointer to an mhl_tx_read_dev_cap
 * structure.  The regNum field of this structure must be filled in by the
 * caller with the index of the Device Capabilities register to be read. Upon
 * return the regValue field of this structure will contain the most recent
 * value read from the specified Device Capabilities register.
 *
 * This IOCTL will return EINVAL if an MHL connection is not established at the
 * time of the call or if the index value passed is invalid.  The driver will
 * return EAGAIN if it is currently in a state where it cannot immediately satisfy
 * the request.  If EAGAIN is returned the caller should wait a short amount of
 * time and retry the request.
 */
#define SII_MHL_GET_DEV_CAP_VALUE \
    _IOWR(IOC_SII_MHL_TYPE, 0x06, struct mhl_tx_read_dev_cap *)


/*
 * IOCTL to write the Scratch Pad registers of an attached MHL device.
 *
 * An application can use this IOCTL to write up to 16 data bytes within the
 * scratch pad register space of an attached MHL device.
 *
 * The parameter passed with this IOCTL is a pointer to an
 * mhl_tx_scratch_pad_access structure.  The offset field of this structure
 * must be filled in by the caller with the starting offset within the
 * Scratch Pad register address space of the 1st register to write.  The length
 * field specifies the total number of bytes passed in the data field.  The
 * data field contains the data bytes to be written.  Per the MHL specification,
 * the write data must be prepended with the Adopter ID of the receiving device.
 * The value passed in the length field must include the Adopter ID bytes.
 *
 * This IOCTL will return EINVAL if an MHL connection is not established at the
 * time of the call or if the offset or length fields contain out of range values.
 * The driver will return EAGAIN if it is currently in a state where it cannot
 * immediately satisfy the request.  If EAGAIN is returned the caller should wait
 * a short amount of time and retry the request.
 */
#define SII_MHL_WRITE_SCRATCH_PAD \
    _IOWR(IOC_SII_MHL_TYPE, 0x07, struct mhl_tx_scratch_pad_access *)


/*
 * IOCTL to read the Scratch Pad registers of an attached MHL device.
 *
 * An application can use this IOCTL to read up to 16 data bytes within the
 * scratch pad register space of an attached MHL device.
 *
 * The parameter passed with this IOCTL is a pointer to an
 * mhl_tx_scratch_pad_access structure.  The offset field of this structure
 * must be filled in by the caller with the starting offset within the Scratch
 * Pad register address space of the 1st register to read.  The length field
 * specifies the total number of bytes to read starting at offset.  Upon
 * successful completion the data field will contain the data bytes read.
 *
 * This IOCTL will return EINVAL if an MHL connection is not established at the
 * time of the call or if the offset or length fields contain out of range values.
 * The driver will return EAGAIN if it is currently in a state where it cannot
 * immediately satisfy the request.  If EAGAIN is returned the caller should wait
 * a short amount of time and retry the request.
 */
#define SII_MHL_READ_SCRATCH_PAD \
    _IOWR(IOC_SII_MHL_TYPE, 0x08, struct mhl_tx_scratch_pad_access *)


/*
 * IOCTL to send a UTF-8 character code
 *
 * An application uses this IOCTL code to send a UTF-8 character code to the
 * downstream display device.
 *
 * The parameter passed with this IOCTL is the UTF-8 character code to be
 * sent to the downstream display device.
 */
#define SII_MHL_UCP_SEND \
    _IOW(IOC_SII_MHL_TYPE, 0x09, __u8)


/*
 * IOCTL to send a UTF-8 character code acknowledgment
 *
 * An application uses this IOCTL code to send a positive acknowledgment for
 * a UTF-8 character code received from the downstream display device and
 * reported to the application via the SII_MHL_GET_MHL_TX_EVENT IOCTL.
 *
 * The parameter passed with this IOCTL is the UTF-8 character code that
 * is being acknowledged.
 */
#define SII_MHL_UCP_SEND_ACK \
    _IOW(IOC_SII_MHL_TYPE, 0x0A, __u8)


/*
 * IOCTL to send a UTF-8 character code error acknowledgment
 *
 * An application uses this IOCTL code to send a negative acknowledgment for
 * an UTF-8 characther code received from the downstream display device and
 * reported to the application via the SII_MHL_GET_MHL_TX_EVENT IOCTL.
 *
 * The parameter passed with this IOCTL is the UTF-8 character code to be
 * negatively acknowledged.
 */
#define SII_MHL_UCP_SEND_ERR_ACK \
    _IOW(IOC_SII_MHL_TYPE, 0x0B, __u8)


#ifdef __cplusplus
}
#endif  /* _defined (__cplusplus) */

#endif /* _defined (MHL_LINUXDRV_IOCTL_H) */
