/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * nfp_dev.h: nCipher PCI HSM linux device declarations
 */

#ifndef NFP_DEV_H
#define NFP_DEV_H

/* Interpretation of the bits of struct nfp_dev.rd_outstanding */
#define WAIT_BIT  0 /* waiting for data */
#define CMPLT_BIT 1 /* completing a read (got data or timing out) */

struct nfp_dev {
	struct list_head list;

	struct nfpcmd_dev const *cmddev;

	struct nfp_cdev common;

	int iosize[6];

	unsigned int irq;

	unsigned char *read_buf;
	dma_addr_t read_dma;

	unsigned char *write_buf;
	dma_addr_t write_dma;

	struct pci_dev *pcidev;

	int busy;
	int ifvers;
	struct timer_list rd_timer;

	wait_queue_head_t rd_queue;
	unsigned long rd_ready;
	unsigned long rd_outstanding;
	int rd_ok;

	wait_queue_head_t wr_queue;
	unsigned long wr_ready;
	unsigned long wr_outstanding;
	int wr_ok;

	spinlock_t spinlock; /* protect this struct */
};

#endif
