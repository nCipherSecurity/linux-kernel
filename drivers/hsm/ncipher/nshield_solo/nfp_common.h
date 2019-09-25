/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * nfp_common.h: word conversions for PCI operations
 */

#ifndef NFP_COMMON_H
#define NFP_COMMON_H

/* endian byte swapping ------------------------------------------- */

/* little endian systems only for now */

#define FROM_LE32_CONFIG(x) (*(x))

#define TO_LE16_MEM(x, y) (*(x) = (y))
#define FROM_LE16_MEM(x) (*(x))

#define TO_LE32_MEM(x, y) (*(x) = (y))
#define FROM_LE32_MEM(x) (*(x))

#endif
