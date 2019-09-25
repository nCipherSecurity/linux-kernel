/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * nfp_error.h: nCipher PCI HSM error handling
 */

#ifndef NFP_ERROR_H
#define NFP_ERROR_H

#define NFP_SUCCESS	0x0
#define NFP_EFAULT	0x1
#define NFP_ENOMEM	0x2
#define NFP_EINVAL	0x3
#define NFP_EIO		0x4
#define NFP_ENXIO	0x5
#define NFP_ENODEV	0x6
#define NFP_EINTR	0x7
#define NFP_ESTARTING	0x8
#define NFP_EAGAIN	0x9
#define NFP_EPOLLING	0xA
#define NFP_EINTERRUPT	0xB
#define NFP_EUNKNOWN	0x100

int nfp_oserr(int nerr);
int     nfp_error(int oerr);

#endif
