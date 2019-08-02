/*
devinit.h: devinit declaration removed from since 3.8 kernel
 * (c) nCipher Security Limited 2019
 */

#ifndef DEVINIT_H_
#define DEVINIT_H_

#include <linux/version.h>

/*
 * Since linux kernel 3.8 the following macros are removed.
 */

#if LINUX_VERSION_CODE >= VERSION(3,8,0)

# ifndef __devinit
#   define __devinit
# endif

# ifndef __devinitdata
#   define __devinitdata
# endif

# ifndef __devexit
#   define __devexit
# endif

# ifndef __devexit_p
#   define __devexit_p
# endif

#endif /* LINUX_VERSION_CODE >= VERSION(3,8,0) */

#endif /* DEVINIT_H_ */
