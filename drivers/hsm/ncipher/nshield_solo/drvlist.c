// SPDX-License-Identifier: GPL-2.0+
/*
 * drvlist.c: nCipher PCI HSM command driver list
 */

#include "osif.h"
#include "i21555.h"
#include "fsl.h"

const struct nfpcmd_dev *nfp_drvlist[] = { &i21555_cmddev, &fsl_t1022_cmddev,
					   NULL };
