/*************************************************************************
 *
 * nfp_cmd.h: nCipher PCI HSM command driver declarations
 *
 *-------------------------------------------------------------------------
 *
 *-------------------------------------------------------------------------
 *
 * Module Description:
 *
 *    This file defines the nCipher PCI HSM command driver interface.
 *
 *-------------------------------------------------------------------------
 *
 * Notes:
 *
 *    Solo cards will provide concrete implementations of this API to
 *    provide card-specific functionality to complete the interface.
 *    PCI drivers.
 *
 *    10/10/2001 jsh  Original
 *
 *-------------------------------------------------------------------------
 *
 *   This document contains proprietary trade secrets of nCipher Security;
 *   its receipt or possession does not convey any right to reproduce,
 *   disclose its contents, or to manufacture, use, or sell anything that
 *   it may describe. Reproduction, disclosure, or use without specific
 *   authorization of nCipher Security is strictly forbidden.
 *
 *              Copyright (C) nCipher Security Limited 2019
 *                       ALL RIGHTS RESERVED
 *     Rights Reserved Under the Copyright Laws of the United States.
 *
 *************************************************************************/

/** @file
 *
 * Defines the nCipher PCI HSM command driver interface.
 * (c) nCipher Security Limited 2019
 */

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef NFPCMD_H
#define NFPCMD_H

#include "nfp_hostif.h"
#include "nfp_error.h"

/* read and write called with userspace buffer */

typedef struct nfpcmd_dev
{
    const char *name;
    unsigned short vendorid, deviceid, sub_vendorid, sub_deviceid;
    unsigned int bar_sizes[NFP_BARSIZES_COUNT];    /* includes IO bit */
    unsigned int flags, max_ifvers;
    nfp_err (*create)(struct nfp_cdev *cdev);
    nfp_err (*destroy)(void * ctx);
    nfp_err (*started)(struct nfp_cdev *cdev, int lock_flag);
    nfp_err (*stopped)(struct nfp_cdev *cdev);
    nfp_err (*open)(void * ctx);
    nfp_err (*close)(void * ctx);
    nfp_err (*isr)(void *ctx, int *handled);
    nfp_err (*write_block)(unsigned int addr, const char *ublock, int len, void *ctx);
    nfp_err (*read_block)(char *ublock, int len, void *ctx, int *rcount);
    nfp_err (*channel_update)(char *data, int len, void *ctx);
    nfp_err (*ensure_reading)(unsigned int addr, int len, void *ctx, int lock_flag);
    nfp_err (*debug)(int cmd, void *ctx);
    nfp_err (*setcontrol)(const nfdev_control_str *control, void *ctx); /* may be NULL */
    nfp_err (*getstatus)(nfdev_status_str *status, void *ctx); /* may be NULL */
} nfpcmd_dev;

#define NFP_CMD_FLG_NEED_IOBUF  0x1
#define NFP_CMD_FLG_NEED_MSI    0x2

/* list of all supported drivers ---------------------------------------- */

extern const nfpcmd_dev *nfp_drvlist[];

extern const nfpcmd_dev i21555_cmddev;
extern const nfpcmd_dev fsl_c293_cmddev;
extern const nfpcmd_dev fsl_p3041_cmddev;
extern const nfpcmd_dev fsl_t1022_cmddev;
extern const nfpcmd_dev bcm5820_cmddev;

#ifndef PCI_BASE_ADDRESS_SPACE_IO
#define PCI_BASE_ADDRESS_SPACE_IO	0x1
#endif

#ifndef PCI_BASE_ADDRESS_SPACE_PREFETCHABLE
#define PCI_BASE_ADDRESS_SPACE_PREFETCHABLE 0x8
#endif

#define NFP_MAXDEV	16

#define NFP_MEMBAR_MASK    ~0xf
#define NFP_IOBAR_MASK     ~0x3
/*
 This masks off the bottom bits of the PCI_CSR_BAR which signify that the
 BAR is an IO BAR rather than a MEM BAR
 */

#define NFP_WITH_LOCK 1
#define NFP_NO_LOCK   0

#endif /* NFP_CMD_H */

#ifdef __cplusplus
}
#endif

/* end of file */
