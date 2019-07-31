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

nfp_dev.h: COMPANY PCI HSM linux device declarations

  * COPYRIGHT

history

09/10/2001 jsh  Original

*/

#ifndef NFP_DEV_H
#define NFP_DEV_H

#include "nfp_fixup.h"
#include "nfp_hostif.h"
#include "nfp_cmd.h"
#include "nfp_osif.h"

/* Interpretation of the bits of nfp_dev.rd_outstanding */
#define WAIT_BIT 0     /* waiting for data */
#define CMPLT_BIT 1    /* completing a read (got data or timing out) */

typedef struct nfp_dev {
    struct list_head list;

    nfpcmd_dev const *cmddev;

    nfp_cdev common;

    int iosize[6];
  
    unsigned int irq;
  
    unsigned char *read_buf;
    dma_addr_t     read_dma;
  
    unsigned char *write_buf;
    dma_addr_t     write_dma;
 
    struct pci_dev *pcidev;
  
    int busy;
    int ifvers;
    struct timer_list rd_timer;


    nfp_wait_queue_head_t rd_queue;
    long unsigned rd_ready;
    long unsigned rd_outstanding;
    int rd_ok;


    nfp_wait_queue_head_t wr_queue;
    long unsigned wr_ready;
    long unsigned wr_outstanding;
    int wr_ok;
  
    spinlock_t spinlock;
  
  
} nfp_dev;

#endif
