/*
 *      This program is free software; you can redistribute it and/or modify
 *      it under the terms of the GNU General Public License as published by
 *      the Free Software Foundation; either version 2 of the License, or
 *      (at your option) any later version.
 *
 *      This program is distributed in the hope that it will be useful,
 *      but WITHOUT ANY WARRANTY; without even the implied warranty of
 *      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *      GNU General Public License for more details.
 *
 *      You should have received a copy of the GNU General Public License
 *      along with this source file; if not, write to the Free Software
 *      Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/*

nfp_osif.h: nCipher PCI HSM OS interface declarations

 * (c) nCipher Security Limited 2019

history

10/10/2001 jsh  Original

*/

#ifndef NFP_OSIF_H
#define NFP_OSIF_H

#include "nfp_hostif.h"
#include "nfp_error.h"

/* implementation choices ----------------------------------------- */

/** ISR strict locking.
 *
 * Define this to choose strict locking for state variables changed
 * within ISR calls. If not defined, we are relying on the implicit
 * memory barriers in sleeping and waiting functions and the
 * pseudo-barriers of the read and write related function calls
 * themselves.
 */
#define NFP_USE_ISR_LOCKING

/* general typedefs ----------------------------------------------- */

typedef volatile unsigned int reg32;
typedef volatile unsigned short reg16;
typedef volatile unsigned char reg8;

/* timeouts ------------------------------------------------------ */

extern void nfp_sleep( int ms );

/* config space access ------------------------------------------------ */

/* return Little Endian 32 bit config register */
extern nfp_err nfp_config_inl( nfp_cdev *pdev, int offset, unsigned int *res );

/* io space access ------------------------------------------------ */

extern unsigned int nfp_inl( nfp_cdev *pdev, int bar, int offset );
extern unsigned short nfp_inw( nfp_cdev *pdev, int bar, int offset );
extern void nfp_outl( nfp_cdev *pdev, int bar, int offset, unsigned int data );
extern void nfp_outw( nfp_cdev *pdev, int bar, int offset, unsigned short data );

/* user and device memory space access ---------------------------- */

/* NB these 2 functions are not guarenteed to be re-entrant for a given device */
extern nfp_err nfp_copy_from_user_to_dev( nfp_cdev *cdev, int bar, int offset, const char *ubuf, int len);
extern nfp_err nfp_copy_to_user_from_dev( nfp_cdev *cdev, int bar, int offset, char *ubuf, int len);

extern nfp_err nfp_copy_from_user( char *kbuf, const char *ubuf, int len );
extern nfp_err nfp_copy_to_user( char *ubuf, const char *kbuf, int len );

extern nfp_err nfp_copy_from_dev( nfp_cdev *cdev, int bar, int offset, char *kbuf, int len );
extern nfp_err nfp_copy_to_dev( nfp_cdev *cdev, int bar, int offset, const char *kbuf, int len);

/* debug ------------------------------------------------------------ */

#define NFP_DBG1	1
#define NFP_DBGE	NFP_DBG1
#define NFP_DBG2	2
#define NFP_DBG3	3
#define NFP_DBG4	4

#ifdef STRANGE_VARARGS
extern void nfp_log();
#else
extern void nfp_log( int severity, const char *format, ...);
#endif

extern int nfp_debug;

#endif
