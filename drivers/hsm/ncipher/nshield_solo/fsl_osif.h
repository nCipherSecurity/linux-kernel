/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * fsl_osif.h: nCipher PCI HSM FSL command operating system interface
 */

#ifndef FSL_OSIF_H
#define FSL_OSIF_H

#include "fsl.h"
#include "nfp_error.h"
#include "nfp_hostif.h"

#ifdef __linux

#include <linux/io.h>

/**
 * Writes a 32 bit word across PCI to the FSL card.
 *
 * @param cdev command device.
 * @param bar base address region id.
 * @param offset offset in bytes from base address.
 * @param value 32 bit value being written.
 */
static inline void fsl_outl(nfp_cdev *cdev, int bar, int offset,
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
static inline uint32_t fsl_inl(nfp_cdev *cdev, int bar, int offset)
{
	return ioread32(cdev->bar[bar] + FSL_DOORBELL_LOCATION + offset);
}

#elif defined(WINVER) || defined(__sun) || defined(__hpux)

#include "nfp_osif.h"

/**
 * Writes a 32 bit word across PCI to the FSL card.
 *
 * @param cdev command device.
 * @param bar base address region id.
 * @param offset offset in bytes from base address.
 * @param value 32 bit value being written.
 */
static void fsl_outl(nfp_cdev *cdev, int bar, int offset, unsigned int value)
{
	nfp_err ne;
	char *data;

	data = (char *)&value;
	ne = nfp_copy_to_dev(cdev, bar, FSL_DOORBELL_LOCATION + offset,
			     (char *)&value, 4);
	if (ne)
		nfp_log(NFP_DBG1, "%s: nfp_copy_to_dev failed", __func__);
}

/**
 * Reads a 32 bit word across PCI from the FSL card.
 *
 * @param cdev command device.
 * @param bar base address region id.
 * @param offset offset in bytes from base address.
 * @returns 32 bit value.
 */
static uint32_t fsl_inl(nfp_cdev *cdev, int bar, int offset)
{
	nfp_err ne;
	unsigned int value;

	ne = nfp_copy_from_dev(cdev, bar, FSL_DOORBELL_LOCATION + offset,
			       (char *)&value, 4);
	if (ne) {
		nfp_log(NFP_DBG1, "%s: nfp_copy_to_dev failed", __func__);
		value = 0;
	}
	return value;
}

#else
#error fsl_osif implementation not defined!
#endif

#endif

/* end of file */
