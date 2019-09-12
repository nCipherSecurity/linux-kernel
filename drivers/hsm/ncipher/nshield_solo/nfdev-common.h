/* SPDX-License-Identifier: GPL-2.0+ */

/** @file nfdev-common.h
 *
 * @brief nFast device driver ioctl struct definition file
 *  see nfdev-linux.h for ioctl number definitions
 */

#ifndef NFDEV_COMMON_H
#define NFDEV_COMMON_H

/**
 * Result of the ENQUIRY ioctl.
 */
struct nfdev_enquiry_str {
	u32 busno; /**< Which bus is the PCI device on. */
	unsigned char slotno; /**< Which slot is the PCI device in. */
	unsigned char reserved[3]; /**< for consistent struct alignment */
};

/**
 * Result of the STATS ioctl.
 */
struct nfdev_stats_str {
	u32 isr; /**< Count interrupts. */
	u32 isr_read; /**< Count read interrupts. */
	u32 isr_write; /**< Count write interrupts. */
	u32 write_fail; /**< Count write failures. */
	u32 write_block; /**< Count blocks written. */
	u32 write_byte; /**< Count bytes written. */
	u32 read_fail; /**< Count read failures. */
	u32 read_block; /**< Count blocks read. */
	u32 read_byte; /**< Count bytes read. */
	u32 ensure_fail; /**< Count read request failures. */
	u32 ensure; /**< Count read requests. */
};

/**
 * Input to the CONTROL ioctl.
 */
struct nfdev_control_str {
	u32 control; /**< Control flags. */
};

/** Control bit indicating host supports MOI control
 */
#define NFDEV_CONTROL_HOST_MOI         0x0001

/** Index of control bits indicating desired mode
 *
 * Desired mode follows the M_ModuleMode enumeration.
 */
#define NFDEV_CONTROL_MODE_SHIFT       1

/** Detect a backwards-compatible control value
 *
 * Returns true if the request control value "makes no difference", i.e.
 * and the failure of an attempt to set it is therefore uninteresting.
 */
#define NFDEV_CONTROL_HARMLESS(c) ((c) <= 1)

/**
 * Result of the STATUS ioctl.
 */
struct nfdev_status_str {
	u32 status; /**< Status flags. */
	char error[8]; /**< Error string. */
};

/** Monitor firmware supports MOI control and error reporting
 */
#define NFDEV_STATUS_MONITOR_MOI       0x0001

/** Application firmware supports MOI control and error reporting
 */
#define NFDEV_STATUS_APPLICATION_MOI   0x0002

/** Application firmware running and supports error reporting
 */
#define NFDEV_STATUS_APPLICATION_RUNNING 0x0004

/** HSM failed
 *
 * Consult error[] for additional information.
 */
#define NFDEV_STATUS_FAILED            0x0008

/** Standard PCI interface. */
#define NFDEV_IF_STANDARD	       0x01

/** PCI interface with read replies pushed from device
 *  via DMA.
 */
#define NFDEV_IF_PCI_PUSH	       0x02

/** PCI interface with read replies pushed from device
 *  and write requests pulled from host via DMA.
 */
#define NFDEV_IF_PCI_PULL 0x03

/** Maximum PCI interface. */
#define NFDEV_IF_MAX_VERS              NFDEV_IF_PCI_PUSH_PULL

/* platform independent base ioctl numbers */

/** Enquiry ioctl.
 *  \return nfdev_enquiry_str describing the attached device.
 */
#define NFDEV_IOCTL_NUM_ENQUIRY        0x01

/** Channel Update ioctl.
 *  \deprecated
 */
#define NFDEV_IOCTL_NUM_CHUPDATE       0x02

/** Ensure Reading ioctl.
 *  Signal a read request to the device.
 *  \param (unsigned int) Length of data to be read.
 */
#define NFDEV_IOCTL_NUM_ENSUREREADING  0x03

/** Device Count ioctl.
 *  Not implemented for on all platforms.
 *  \return (int) the number of attached devices.
 */
#define NFDEV_IOCTL_NUM_DEVCOUNT       0x04

/** Internal Debug ioctl.
 *  Not implemented in release drivers.
 */
#define NFDEV_IOCTL_NUM_DEBUG          0x05

/** PCI Interface Version ioctl.
 *  \param (int) Maximum PCI interface version
 *   supported by the user of the device.
 */
#define NFDEV_IOCTL_NUM_PCI_IFVERS     0x06

/** Statistics ioctl.
 *  \return nfdev_enquiry_str describing the attached device.
 */
#define NFDEV_IOCTL_NUM_STATS          0x07

/** Module control ioctl
 * \param (nfdev_control_str) Value to write to HSM control register
 */
#define NFDEV_IOCTL_NUM_CONTROL        0x08

/** Module state ioctl
 * \return (nfdev_status_str) Values read from HSM status/error registers
 */
#define NFDEV_IOCTL_NUM_STATUS         0x09

#endif
