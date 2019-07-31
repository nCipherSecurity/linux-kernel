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

nfp_hostif.h: COMPANY PCI HSM host interface declarations

  * COPYRIGHT

history

10/10/2001 jsh  Original

*/

#ifndef NFP_HOSTIF_H
#define NFP_HOSTIF_H

#include "nfdev-common.h"

#define NFP_BARSIZES_COUNT 6
#define NFP_BARSIZES_MASK ~0xF

struct nfp_dev;

/* common device structure */

typedef struct nfp_cdev {
  unsigned char *bar[NFP_BARSIZES_COUNT];
  void *extra[NFP_BARSIZES_COUNT];

  int busno;
  int slotno;

  void *cmdctx;

  char *iobuf;

  struct nfp_dev* dev;

  struct nfdev_stats_str stats;
  int active_bar;
  int created;
  int conn_status;
  int detection_type; 
} nfp_cdev;

/* callbacks from command drivers -------------------------------------- */

void nfp_read_complete(  struct nfp_dev *pdev, int ok);
void nfp_write_complete( struct nfp_dev *pdev, int ok);

#define NFP_READ_MAX (8 * 1024)
#define NFP_WRITE_MAX (8 * 1024)
#define NFP_READBUF_SIZE (NFP_READ_MAX + 8)
#define NFP_WRITEBUF_SIZE (NFP_WRITE_MAX + 8)
#define NFP_TIMEOUT_SEC 20
#define NFP_TIMEOUT_MSEC (1000 * NFP_TIMEOUT_SEC)
#define NFP_DMA_NBYTES_OFFSET (4)
#define NFP_DMA_ADDRESS_OFFSET (8)

#define NFP_DRVNAME "COMPANY nFast PCI driver"

#define NFP_CLSNAME "nfp"

#endif
