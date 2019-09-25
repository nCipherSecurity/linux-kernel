/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * fsl_osif.h: nCipher PCI HSM FSL command operating system interface
 */

#ifndef FSL_OSIF_H
#define FSL_OSIF_H

#include <linux/io.h>

/**
 * Writes a 32 bit word across PCI to the FSL card.
 *
 * @param cdev command device.
 * @param bar base address region id.
 * @param offset offset in bytes from base address.
 * @param value 32 bit value being written.
 */
static inline void fsl_outl(struct nfp_cdev *cdev, int bar, int offset,
			    unsigned int value)
{
	iowrite32(value, cdev->bar[bar] + FSL_DOORBELL_LOCATION + offset);
}

/**
 * Reads a 32 bit word across PCI from the FSL card.
 *
 * @param cdev command device.
 * @param bar base address region id.
 * @param offset offset in bytes from base address.
 * @returns 32 bit value.
 */
static inline uint32_t fsl_inl(struct nfp_cdev *cdev, int bar, int offset)
{
	return ioread32(cdev->bar[bar] + FSL_DOORBELL_LOCATION + offset);
}

#endif
/* end of file */
