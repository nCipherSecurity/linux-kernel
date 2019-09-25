/* SPDX-License-Identifier: GPL-2.0+ */
/*
 *
 * nfp_hostif.h: nCipher PCI HSM host interface declarations
 *
 */

#ifndef NFP_HOSTIF_H
#define NFP_HOSTIF_H

#define NFP_BARSIZES_COUNT 6
#define NFP_BARSIZES_MASK ~0xF

struct nfp_dev;

/* common device structure */

struct nfp_cdev {
	unsigned char *bar[NFP_BARSIZES_COUNT];
	void *extra[NFP_BARSIZES_COUNT];

	int busno;
	int slotno;

	void *cmdctx;

	char *iobuf;

	struct nfp_dev *dev;

	struct nfdev_stats_str stats;
	int active_bar;
	int created;
	int conn_status;
	int detection_type;
};

/* callbacks from command drivers -------------------------------------- */

void nfp_read_complete(struct nfp_dev *pdev, int ok);
void nfp_write_complete(struct nfp_dev *pdev, int ok);

#define NFP_READ_MAX (8 * 1024)
#define NFP_WRITE_MAX (8 * 1024)
#define NFP_READBUF_SIZE (NFP_READ_MAX + 8)
#define NFP_WRITEBUF_SIZE (NFP_WRITE_MAX + 8)
#define NFP_TIMEOUT_SEC 20
#define NFP_TIMEOUT_MSEC (1000 * NFP_TIMEOUT_SEC)
#define NFP_DMA_NBYTES_OFFSET (4)
#define NFP_DMA_ADDRESS_OFFSET (8)

#define NFP_DRVNAME "nCipher nFast PCI driver"
#define NFP_CLSNAME "nshield_solo"

#endif
