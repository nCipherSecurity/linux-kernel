// SPDX-License-Identifier: GPL-2.0+
/*
 * drvlist.c: nCipher PCI HSM command driver list
 * (c) nCipher Security Limited 2019
 */

#include "nfp_common.h"
#include "nfp_fixup.h"
#include "nfp_cmd.h"

const struct nfpcmd_dev *nfp_drvlist[] = { &i21555_cmddev, &fsl_c293_cmddev,
				    &fsl_p3041_cmddev, &fsl_t1022_cmddev,
				    NULL };
