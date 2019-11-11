/* SPDX-License-Identifier: GPL-2.0+ */
/*
 *
 * osif.h: nCipher PCI HSM OS interface declarations
 *
 */

#ifndef NFP_OSIF_H
#define NFP_OSIF_H

#include <linux/version.h>
#include <linux/types.h>
#include <linux/time.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/ptrace.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/poll.h>
#include <linux/nshield_solo.h>

/* little endian systems only for now */

#define FROM_LE32_CONFIG(x) (*(x))

#define TO_LE16_MEM(x, y) (*(x) = (y))
#define FROM_LE16_MEM(x) (*(x))

#define TO_LE32_MEM(x, y) (*(x) = (y))
#define FROM_LE32_MEM(x) (*(x))

#define VERSION(ver, rel, seq) (((ver) << 16) | ((rel) << 8) | (seq))
#define NFP_MAJOR 176 /* "nCipher nFast PCI crypto accelerator" */

#define COPY_FROM_USER(DST, SRC, LEN, error)                                   \
	(error = copy_from_user(DST, SRC, LEN) ? -EFAULT : 0)
#define COPY_TO_USER(DST, SRC, LEN, error)                                     \
	(error = copy_to_user(DST, SRC, LEN) ? -EFAULT : 0)
#define INODE_FROM_FILE(file) ((file)->f_path.dentry->d_inode)

#define NFP_TIMEOUT ((NFP_TIMEOUT_SEC) * HZ)
#define NFP_BARSIZES_COUNT 6
#define NFP_BARSIZES_MASK ~0xF

/* Interpretation of the bits of struct nfp_dev.rd_outstanding */
#define WAIT_BIT  0 /* waiting for data */
#define CMPLT_BIT 1 /* completing a read (got data or timing out) */

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

/* Per-device-type command handlers */
struct nfpcmd_dev {
	const char *name;
	unsigned short vendorid, deviceid, sub_vendorid, sub_deviceid;
	unsigned int bar_sizes[NFP_BARSIZES_COUNT]; /* includes IO bit */
	unsigned int flags, max_ifvers;

	int (*create)(struct nfp_cdev *cdev);
	int (*destroy)(void *ctx);
	int (*started)(struct nfp_cdev *cdev, int lock_flag);
	int (*stopped)(struct nfp_cdev *cdev);
	int (*open)(void *ctx);
	int (*close)(void *ctx);
	int (*isr)(void *ctx, int *handled);
	int (*write_block)(unsigned int addr, const char *ublock, int len,
			   void *ctx);
	int (*read_block)(char *ublock, int len, void *ctx, int *rcount);
	int (*channel_update)(char *data, int len, void *ctx);
	int (*ensure_reading)(unsigned int addr, int len, void *ctx,
			      int lock_flag);
	int (*debug)(int cmd, void *ctx);
	int (*setcontrol)(const struct nfdev_control_str *control,
			  void *ctx); /* may be NULL */
	int (*getstatus)(struct nfdev_status_str *status,
			 void *ctx); /* may be NULL */
};

/* per-instance device structure */
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

#define NFP_CMD_FLG_NEED_IOBUF 0x1
#define NFP_CMD_FLG_NEED_MSI   0x2

/* list of all supported drivers ---------------------------------------- */

extern const struct nfpcmd_dev *nfp_drvlist[];

extern const struct nfpcmd_dev i21555_cmddev;
extern const struct nfpcmd_dev fsl_t1022_cmddev;

#ifndef PCI_BASE_ADDRESS_SPACE_IO
#define PCI_BASE_ADDRESS_SPACE_IO	0x1
#endif

#ifndef PCI_BASE_ADDRESS_SPACE_PREFETCHABLE
#define PCI_BASE_ADDRESS_SPACE_PREFETCHABLE 0x8
#endif

#define NFP_MAXDEV      16

#define NFP_MEMBAR_MASK ~0xf
#define NFP_IOBAR_MASK  ~0x3
/*
 * This masks off the bottom bits of the PCI_CSR_BAR which signify that the
 * BAR is an IO BAR rather than a MEM BAR
 */

#define NFP_WITH_LOCK 1
#define NFP_NO_LOCK   0

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

/* timeouts ------------------------------------------------------ */

void nfp_sleep(int ms);

/* config space access ------------------------------------------------ */

/* return Little Endian 32 bit config register */
int nfp_config_inl(struct nfp_cdev *pdev, int offset, unsigned int *res);

/* io space access ------------------------------------------------ */

unsigned int nfp_inl(struct nfp_cdev *pdev, int bar, int offset);
unsigned short nfp_inw(struct nfp_cdev *pdev, int bar, int offset);
void nfp_outl(struct nfp_cdev *pdev, int bar, int offset, unsigned int data);
void nfp_outw(struct nfp_cdev *pdev, int bar, int offset, unsigned short data);

/* user and device memory space access ---------------------------- */

/*
 * NB these 2 functions are not guaranteed to be re-entrant for a given device
 */
int nfp_copy_from_user_to_dev(struct nfp_cdev *cdev, int bar, int offset,
			      const char *ubuf, int len);
int nfp_copy_to_user_from_dev(struct nfp_cdev *cdev, int bar, int offset,
			      char *ubuf, int len);

int nfp_copy_from_user(char *kbuf, const char *ubuf, int len);
int nfp_copy_to_user(char *ubuf, const char *kbuf, int len);

int nfp_copy_from_dev(struct nfp_cdev *cdev, int bar, int offset,
		      char *kbuf, int len);
int nfp_copy_to_dev(struct nfp_cdev *cdev, int bar, int offset,
		    const char *kbuf, int len);

/* debug ------------------------------------------------------------ */

#define NFP_DBG1	1
#define NFP_DBGE	NFP_DBG1
#define NFP_DBG2	2
#define NFP_DBG3	3
#define NFP_DBG4	4

void nfp_log(int severity, const char *format, ...);
extern int nfp_debug;

/* Error codes & conversions */

#define NFP_SUCCESS	0x0
#define NFP_EFAULT	0x1
#define NFP_ENOMEM	0x2
#define NFP_EINVAL	0x3
#define NFP_EIO		0x4
#define NFP_ENXIO	0x5
#define NFP_ENODEV	0x6
#define NFP_EINTR	0x7
#define NFP_ESTARTING	0x8
#define NFP_EAGAIN	0x9
#define NFP_EPOLLING	0xA
#define NFP_EINTERRUPT	0xB
#define NFP_EUNKNOWN	0x100

int nfp_oserr(int nerr);
int     nfp_error(int oerr);

#endif /* NFP_OSIF_H */
