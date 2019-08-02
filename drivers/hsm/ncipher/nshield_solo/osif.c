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

osif.c: nCipher PCI HSM OS interface

 * (c) nCipher Security Limited 2019

history

09/10/2001 jsh  Original

*/

#include "nfp_common.h"
#include "nfp_error.h"
#include "nfp_fixup.h"
#include "nfp_hostif.h"
#include "nfp_osif.h"
#include "nfp_error.h"
#include "autoversion.h"
#include "nfp_dev.h"

#include <linux/spinlock.h>
#include <linux/slab.h>

void nfp_sleep( int ms ) {
  DEFINE_WAIT(wait);
  nfp_wait_queue_head_t q;

  nfp_init_waitqueue_head( &q );

  prepare_to_wait(&q, &wait, TASK_UNINTERRUPTIBLE);
  schedule_timeout((ms * HZ) / 1000);
  finish_wait(&q, &wait);
}

nfp_err nfp_config_inl( nfp_cdev *pdev, int offset, unsigned int *res ) {
  if(!pdev->dev || !pdev->dev->pcidev)
    return NFP_ENODEV;
  pci_read_config_dword( pdev->dev->pcidev, offset, res);
  return 0;
}

/* user space memory access ---------------------------------- */

nfp_err nfp_copy_from_user( char *kbuf, const char *ubuf, int len) {
  int oserr;
  long task_state;
  task_state = current->state;
  COPY_FROM_USER(kbuf, ubuf, len, oserr);
  current->state = task_state;
  nfr( oserr );
}

nfp_err nfp_copy_to_user( char *ubuf, const char *kbuf, int len) {
  int oserr;
  long task_state;
  task_state = current->state;
  COPY_TO_USER(ubuf, kbuf, len, oserr);
  current->state = task_state;
  nfr( oserr );
}

nfp_err nfp_copy_from_user_to_dev( nfp_cdev *cdev, int bar, int offset, const char *ubuf, int len) {
  int oserr;
  long task_state;
  task_state = current->state;
  COPY_FROM_USER(cdev->bar[bar] + offset, ubuf, len, oserr);
  current->state = task_state;
  nfr( oserr );
}

nfp_err nfp_copy_to_user_from_dev( nfp_cdev *cdev, int bar, int offset, char *ubuf, int len) {
  int oserr;
  long task_state;
  task_state = current->state;
  COPY_TO_USER(ubuf, cdev->bar[bar] + offset, len, oserr);
  current->state = task_state;
  nfr( oserr );
}

nfp_err nfp_copy_from_dev( nfp_cdev *cdev, int bar, int offset, char *kbuf, int len) {
  memcpy( kbuf, cdev->bar[bar] + offset, len);
  return NFP_SUCCESS;
}

nfp_err nfp_copy_to_dev( nfp_cdev *cdev, int bar, int offset, const char *kbuf, int len) {
  memcpy( cdev->bar[bar] + offset, kbuf, len);
  return NFP_SUCCESS;
}

/* pci fixed length accessors. The below functions are used predominantly
 * to access CSR registers in pci memory space. */
unsigned int nfp_inl( nfp_cdev *pdev, int bar, int offset ) {
  nfp_log( NFP_DBG3, "nfp_inl: addr %p", pdev->bar[bar] + offset);
  return ioread32( pdev->bar[bar] + offset );
}

unsigned short nfp_inw( nfp_cdev *pdev, int bar, int offset ) {
  nfp_log( NFP_DBG3, "nfp_inl: addr %p", pdev->bar[bar] + offset);
  return ioread16( pdev->bar[bar] + offset );
}

void nfp_outl( nfp_cdev *pdev, int bar, int offset, unsigned int data ) {
  nfp_log( NFP_DBG3, "nfp_outl: addr %p, data %x", pdev->bar[bar] + offset, data);
  iowrite32( data, pdev->bar[bar] + offset );
}

void nfp_outw( nfp_cdev *pdev, int bar, int offset, unsigned short data ) {
  nfp_log( NFP_DBG3, "nfp_outl: addr %p, data %x", pdev->bar[bar] + offset, data);
  iowrite16( data, pdev->bar[bar] + offset );
}

/* logging ---------------------------------------------------- */

void nfp_log( int level, const char *fmt, ...)
{
  va_list ap;

  switch (level) {
  case NFP_DBG4: if (nfp_debug < 4) break;
  /*FALLTHROUGH*/
  case NFP_DBG3: if (nfp_debug < 3) break;
  /*FALLTHROUGH*/
  case NFP_DBG2: if (nfp_debug < 2) break;
  /*FALLTHROUGH*/
  case NFP_DBG1: if (nfp_debug < 1) break;
  /*FALLTHROUGH*/
  default:
    printk (VERSION_COMPNAME " " VERSION_STRING ": ");
    va_start(ap, fmt);
    (void) vprintk(fmt, ap);
    va_end(ap);
    printk ("\n");
    break;
  }
}

struct errstr {
  int oserr;
  nfp_err nferr;
};

static struct errstr errtab[] = {
  { -EFAULT, NFP_EFAULT },
  { -ENOMEM, NFP_ENOMEM },
  { -EINVAL, NFP_EINVAL },
  { -EIO, NFP_EIO },
  { -ENXIO, NFP_ENXIO },
  { -ENODEV, NFP_ENODEV },
  { 0, 0 }
};

nfp_err nfp_error( int oserr ) {
  struct errstr *perr;
  if(!oserr)
    return 0;
  perr= errtab;
  while(perr->nferr) {
   if(perr->oserr == oserr)
     return perr->nferr;
   perr++;
  }
  return NFP_EUNKNOWN;
}

int nfp_oserr( nfp_err nferr )
{
  struct errstr *perr;
  if( nferr == NFP_SUCCESS)
    return 0;
  perr= errtab;
  while(perr->nferr) {
   if(perr->nferr == nferr)
     return perr->oserr;
   perr++;
  }
  return -EIO;
}
