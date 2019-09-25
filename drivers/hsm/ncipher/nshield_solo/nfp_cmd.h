/* SPDX-License-Identifier: GPL-2.0+ */
/*************************************************************************
 *
 * nfp_cmd.h: nCipher PCI HSM command driver declarations
 *
 * This file defines the nCipher PCI HSM command driver interface.
 *
 * Solo cards will provide concrete implementations of this API to
 * provide card-specific functionality to complete the interface.
 * PCI drivers.
 *
 *************************************************************************/

#ifndef NFPCMD_H
#define NFPCMD_H

/* read and write called with userspace buffer */

struct nfpcmd_dev {
	const char *name;
	unsigned short vendorid, deviceid, sub_vendorid, sub_deviceid;
	unsigned int bar_sizes[NFP_BARSIZES_COUNT]; /* includes IO bit */
	unsigned int flags, max_ifvers;

	int (*create)(struct nfp_cdev *cdev);
	int (*destroy)(void *ctx);
	int (*started)(struct nfp_cdev *cdev, int lock_flag);
	int (*stopped)(struct nfp_cdev *cdev);
	int (*open)(void *ctx);
	int (*close)(void *ctx);
	int (*isr)(void *ctx, int *handled);
	int (*write_block)(unsigned int addr, const char *ublock, int len,
			   void *ctx);
	int (*read_block)(char *ublock, int len, void *ctx, int *rcount);
	int (*channel_update)(char *data, int len, void *ctx);
	int (*ensure_reading)(unsigned int addr, int len, void *ctx,
			      int lock_flag);
	int (*debug)(int cmd, void *ctx);
	int (*setcontrol)(const struct nfdev_control_str *control,
			  void *ctx); /* may be NULL */
	int (*getstatus)(struct nfdev_status_str *status,
			 void *ctx); /* may be NULL */
};

#define NFP_CMD_FLG_NEED_IOBUF 0x1
#define NFP_CMD_FLG_NEED_MSI   0x2

/* list of all supported drivers ---------------------------------------- */

extern const struct nfpcmd_dev *nfp_drvlist[];

extern const struct nfpcmd_dev i21555_cmddev;
extern const struct nfpcmd_dev fsl_c293_cmddev;
extern const struct nfpcmd_dev fsl_p3041_cmddev;
extern const struct nfpcmd_dev fsl_t1022_cmddev;
extern const struct nfpcmd_dev bcm5820_cmddev;

#ifndef PCI_BASE_ADDRESS_SPACE_IO
#define PCI_BASE_ADDRESS_SPACE_IO	0x1
#endif

#ifndef PCI_BASE_ADDRESS_SPACE_PREFETCHABLE
#define PCI_BASE_ADDRESS_SPACE_PREFETCHABLE 0x8
#endif

#define NFP_MAXDEV      16

#define NFP_MEMBAR_MASK ~0xf
#define NFP_IOBAR_MASK  ~0x3
/*
 * This masks off the bottom bits of the PCI_CSR_BAR which signify that the
 * BAR is an IO BAR rather than a MEM BAR
 */

#define NFP_WITH_LOCK 1
#define NFP_NO_LOCK   0

#endif /* NFP_CMD_H */

/* end of file */
