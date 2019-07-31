/*************************************************************************
 *
 * Project:      NGSolo
 *
 *-------------------------------------------------------------------------
 *
 *-------------------------------------------------------------------------
 *
 * Module Description:
 *
 *    This file declares FSL device configurations.
 *
 *-------------------------------------------------------------------------
 *
 * Notes:
 *
 *    This file is duplicated in the host and card repositories. It is
 *    essential that both copies match!
 *
 *-------------------------------------------------------------------------
 *
 *   This document contains proprietary trade secrets of COMPANY;
 *   its receipt or possession does not convey any right to reproduce,
 *   disclose its contents, or to manufacture, use, or sell anything that
 *   it may describe. Reproduction, disclosure, or use without specific
 *   authorization of COMPANY is strictly forbidden.
 *
 *              Copyright (C) 2015 by COMPANY
 *                       ALL RIGHTS RESERVED
 *     Rights Reserved Under the Copyright Laws of the United States.
 *
 *************************************************************************/

/** @file
 *
 * Declares FSL device configurations.
 */

#ifdef __cplusplus
extern "C"
{
#endif


#ifndef FSL_H
#define FSL_H

#include "nfpci.h"

/* PCI FSL definitions */

#ifndef PCI_VENDOR_ID_FREESCALE
#define PCI_VENDOR_ID_FREESCALE           0x1957
#endif

#ifndef PCI_DEVICE_ID_FREESCALE_C293
#define PCI_DEVICE_ID_FREESCALE_C293      0x0800
#endif

#ifndef PCI_DEVICE_ID_FREESCALE_P3041
#define PCI_DEVICE_ID_FREESCALE_P3041     0x041e
#endif

#ifndef PCI_DEVICE_ID_FREESCALE_T1022
#define PCI_DEVICE_ID_FREESCALE_T1022     0x082c
#endif

#ifndef PCI_VENDOR_ID_NCIPHER
#define PCI_VENDOR_ID_NCIPHER             0x0100
#endif

#ifndef PCI_SUBSYSTEM_ID_NFAST_REV1
#define PCI_SUBSYSTEM_ID_NFAST_REV1       0x0100
#endif

#define FSL_CFG_SEC_CMD_STATUS            0x4C
#define FSL_CFG_CMD_MASTER                0x10

#define FSL_MEMBAR                        1

/* NFAST extended PCI register definitions */

#define NFPCI_OFFSET_JOBS_DS              0x00020000

/* Interrupts from device to host */

#define NFAST_INT_DEVICE_CLR              0x00000000
#define NFAST_INT_DEVICE_CHECK_OK         0x00000100
#define NFAST_INT_DEVICE_CHECK_FAILED     0x00000200
#define NFAST_INT_DEVICE_POLL             0x00000300
#define NFAST_INT_DEVICE_PCI_DOWN         0x00000400

/* Interrupts from host to device */

#define NFAST_INT_HOST_CLR                0x00000000

/* PCI FSL register definitions */

#define FSL_LENGTH                        NFPCI_RAM_MINSIZE_JOBS
#define FSL_MEMSIZE                       NFPCI_RAM_MINSIZE_KERN

#define FSL_DOORBELL_LOCATION             (FSL_MEMSIZE - 0x100)

#define FSL_OFFSET_DOORBELL_RD_CMD        0x00u
#define FSL_OFFSET_DOORBELL_WR_CMD        0x04u
#define FSL_OFFSET_DOORBELL_RD_STATUS     0x08u
#define FSL_OFFSET_DOORBELL_WR_STATUS     0x0Cu
#define FSL_OFFSET_DOORBELL_CS_STATUS     0x10u

#define FSL_OFFSET_REGISTER_CONTROL       0x20u
#define FSL_OFFSET_REGISTER_STATUS        0x24u
#define FSL_OFFSET_REGISTER_ERROR_LO      0x28u
#define FSL_OFFSET_REGISTER_ERROR_HI      0x2Cu
#define FSL_OFFSET_DOORBELL_POLLING       0x30u

#define FSL_MAGIC                         0x12345678u

/** Monitor firmware supports MOI control and error reporting */
#define NFDEV_STATUS_MONITOR_MOI          0x0001

/** Application firmware supports MOI control and error reporting */
#define NFDEV_STATUS_APPLICATION_MOI      0x0002

/** Application firmware running and supports error reporting */
#define NFDEV_STATUS_APPLICATION_RUNNING  0x0004

#endif

#ifdef __cplusplus
}
#endif

/* end of file */
