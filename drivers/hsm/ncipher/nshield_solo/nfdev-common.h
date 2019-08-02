/** \file nfdev-common.h
 *
 * (C) nCipher Security Limited 2019
 *
 * The copyright in this software is the property of nCipher Security Limited.
 * This software may not be used, sold, licensed, disclosed, transferred, copied,
 * modified or reproduced in whole or in part or in any manner or form other than
 * in accordance with the licence agreement provided with this software or
 * otherwise without the prior written consent of nCipher Security Limited.
 *
 * \brief nFast device driver (not generic SCSI) ioctl struct definition file
 *  include NFDEV-$(system) for ioctl number definitions
 *
 *  1998.07.13	jsh	Started
 *
 * 
 */

#ifndef NFDEV_COMMON_H
#define NFDEV_COMMON_H

#if defined(__KERNEL__) || defined(__OpenBSD_kernel__)
/* uint32_t is known about - don't try to define it */
#else
/* We need to defined uint32_t somehow */
#if defined(_WIN32)
/* int is always 32-bits */
typedef unsigned int uint32_t;
#else
#if defined(__OpenBSD__)
#include <sys/types.h>
#else
/* inttypes.h eventually defines uint32_t */
#include <inttypes.h>
#endif
#endif
#endif

/**
 * tchar, plus a shim for non-Windows OSes.
 */
#if defined(_WIN32)
#include <tchar.h>
#else
#define TCHAR char
#define _T(x) (x)
#endif

/**
 * Result of the ENQUIRY ioctl.
 */
typedef struct nfdev_enquiry_str {
  uint32_t  busno; /**< Which bus is the PCI device on. */
  unsigned char slotno; /**< Which slot is the PCI device in. */
  unsigned char reserved[3]; /**< for consistant struct alignment */
} nfdev_enquiry_str;

/**
 * Result of the STATS ioctl.
 */
typedef struct nfdev_stats_str {
  uint32_t  isr; /**< Count interrupts. */
  uint32_t  isr_read; /**< Count read interrupts. */
  uint32_t  isr_write; /**< Count write interrupts. */
  uint32_t  write_fail; /**< Count write failures. */
  uint32_t  write_block; /**< Count blocks written. */
  uint32_t  write_byte; /**< Count bytes written. */
  uint32_t  read_fail; /**< Count read failures. */
  uint32_t  read_block; /**< Count blocks read. */
  uint32_t  read_byte; /**< Count bytes read. */
  uint32_t  ensure_fail; /**< Count read request failures. */
  uint32_t  ensure; /**< Count read requests. */
} nfdev_stats_str;

/**
 * Input to the CONTROL ioctl.
 */
typedef struct nfdev_control_str {
  uint32_t  control; /**< Control flags. */
} nfdev_control_str;

/** Control bit indicating host supports MOI control */
#define NFDEV_CONTROL_HOST_MOI 0x0001

/** Index of control bits indicating desired mode
 *
 * Desired mode follows the M_ModuleMode enumeration.
 */
#define NFDEV_CONTROL_MODE_SHIFT 1

/** Detect a backwards-compatible control value
 *
 * Returns true if the request control value "makes no difference", i.e.
 * and the failure of an attempt to set it is therefore uninteresting.
 */
#define NFDEV_CONTROL_HARMLESS(c) ((c) <= 1)

/**
 * Result of the STATUS ioctl.
 */
typedef struct nfdev_status_str {
  uint32_t  status; /**< Status flags. */
  char      error[8]; /**< Error string. */
} nfdev_status_str;

/** Monitor firmware supports MOI control and error reporting */
#define NFDEV_STATUS_MONITOR_MOI 0x0001

/** Application firmware supports MOI control and error reporting */
#define NFDEV_STATUS_APPLICATION_MOI 0x0002

/** Application firmware running and supports error reporting */
#define NFDEV_STATUS_APPLICATION_RUNNING 0x0004

/** HSM failed
 *
 * Consult error[] for additional information.
 */
#define NFDEV_STATUS_FAILED 0x0008

/** Standard PCI interface. */
#define NFDEV_IF_STANDARD	0x01

/** PCI interface with read replies pushed from device
 *  via DMA.
 */
#define NFDEV_IF_PCI_PUSH	0x02

/** PCI interface with read replies pushed from device
 *  and write requests pulled from host via DMA.
 */
#define NFDEV_IF_PCI_PULL 0x03

/** Maximum PCI interface. */
#define NFDEV_IF_MAX_VERS      NFDEV_IF_PCI_PUSH_PULL

/* platform independant base ioctl numbers */

/** Enquiry ioctl.
 *  \return nfdev_enquiry_str describing the attached device. */
#define NFDEV_IOCTL_NUM_ENQUIRY        0x01

/** Channel Update ioctl.
 *  \deprecated */
#define NFDEV_IOCTL_NUM_CHUPDATE       0x02

/** Ensure Reading ioctl.
 *  Signal a read request to the device.
 *  \param (unsigned int) Length of data to be read.
 */
#define NFDEV_IOCTL_NUM_ENSUREREADING  0x03

/** Device Count ioctl.
 *  Not implemented for on all platforms.
 *  \return (int) the number of attached devices. */
#define NFDEV_IOCTL_NUM_DEVCOUNT       0x04

/** Internal Debug ioctl.
 *  Not implemented in release drivers. */
#define NFDEV_IOCTL_NUM_DEBUG          0x05

/** PCI Interface Version ioctl.
 *  \param (int) Maximum PCI interface version
 *   supported by the user of the device. */
#define NFDEV_IOCTL_NUM_PCI_IFVERS     0x06

/** Statistics ioctl.
 *  \return nfdev_enquiry_str describing the attached device. */
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
