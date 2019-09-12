/*
 *
 * nfp.h: nCipher PCI HSM Linux OS interface declarations
 *
 * (c) nCipher Security Limited 2019
 *
 * history
 *
 * 09/10/2001 jsh  Original
 *
 */

#ifndef NFP_COMMON_H
#define NFP_COMMON_H

#include <linux/types.h>

#define DEFINE_NFPCI_PACKED_STRUCTS
#include "nfpci.h"
#include "nfdev-linux.h"

typedef int oserr_t;

/* endian byte sex swapping ------------------------------------------- */

/* little endian systems only for now */

#define FROM_LE32_CONFIG(x) (*(x))

#define TO_LE16_MEM(x, y) (*(x) = (y))
#define FROM_LE16_MEM(x) (*(x))

#define TO_LE32_MEM(x, y) (*(x) = (y))
#define FROM_LE32_MEM(x) (*(x))

#endif
