/*

nfp.h: COMPANY PCI HSM Linux OS interface declarations

  * COPYRIGHT

history

09/10/2001 jsh  Original

*/

#ifndef NFP_COMMON_H
#define NFP_COMMON_H

#include <linux/types.h>

typedef u_int32_t UINT32;    
typedef u_int8_t BYTE;      

#define DEFINE_NFPCI_PACKED_STRUCTS
#include "nfpci.h"
#include "nfdev-linux.h"

typedef int oserr_t;

/* endian byte sex swapping ------------------------------------------- */

/* little endian systems only for now */

#define FROM_LE32_CONFIG(x) (*x)

#define TO_LE16_MEM(x,y) (*x=y)
#define FROM_LE16_MEM(x) (*x)

#define TO_LE32_MEM(x,y) (*x=y)
#define FROM_LE32_MEM(x) (*x)

#endif
