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

/* Device ioctl struct definitions */

/* Result of the ENQUIRY ioctl. */
struct nfdev_enquiry_str {
	u32 busno; /**< Which bus is the PCI device on. */
	unsigned char slotno; /**< Which slot is the PCI device in. */
	unsigned char reserved[3]; /**< for consistent struct alignment */
};

/* Result of the STATS ioctl. */
struct nfdev_stats_str {
	u32 isr; /**< Count interrupts. */
	u32 isr_read; /**< Count read interrupts. */
	u32 isr_write; /**< Count write interrupts. */
	u32 write_fail; /**< Count write failures. */
	u32 write_block; /**< Count blocks written. */
	u32 write_byte; /**< Count bytes written. */
	u32 read_fail; /**< Count read failures. */
	u32 read_block; /**< Count blocks read. */
	u32 read_byte; /**< Count bytes read. */
	u32 ensure_fail; /**< Count read request failures. */
	u32 ensure; /**< Count read requests. */
};

/* Result of the STATUS ioctl. */
struct nfdev_status_str {
	u32 status; /**< Status flags. */
	char error[8]; /**< Error string. */
};

/* Input to the CONTROL ioctl. */
struct nfdev_control_str {
	u32 control; /**< Control flags. */
};

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
extern const struct nfpcmd_dev fsl_c293_cmddev;
extern const struct nfpcmd_dev fsl_p3041_cmddev;
extern const struct nfpcmd_dev fsl_t1022_cmddev;
extern const struct nfpcmd_dev bcm5820_cmddev;

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

/** Control bit indicating host supports MOI control
 */
#define NFDEV_CONTROL_HOST_MOI         0x0001

/** Index of control bits indicating desired mode
 *
 * Desired mode follows the M_ModuleMode enumeration.
 */
#define NFDEV_CONTROL_MODE_SHIFT       1

/** Detect a backwards-compatible control value
 *
 * Returns true if the request control value "makes no difference", i.e.
 * and the failure of an attempt to set it is therefore uninteresting.
 */
#define NFDEV_CONTROL_HARMLESS(c) ((c) <= 1)

/** Monitor firmware supports MOI control and error reporting
 */
#define NFDEV_STATUS_MONITOR_MOI       0x0001

/** Application firmware supports MOI control and error reporting
 */
#define NFDEV_STATUS_APPLICATION_MOI   0x0002

/** Application firmware running and supports error reporting
 */
#define NFDEV_STATUS_APPLICATION_RUNNING 0x0004

/** HSM failed
 *
 * Consult error[] for additional information.
 */
#define NFDEV_STATUS_FAILED            0x0008

/** Standard PCI interface. */
#define NFDEV_IF_STANDARD	       0x01

/** PCI interface with read replies pushed from device
 *  via DMA.
 */
#define NFDEV_IF_PCI_PUSH	       0x02

/** PCI interface with read replies pushed from device
 *  and write requests pulled from host via DMA.
 */
#define NFDEV_IF_PCI_PULL 0x03

/** Maximum PCI interface. */
#define NFDEV_IF_MAX_VERS              NFDEV_IF_PCI_PUSH_PULL

/* platform independent base ioctl numbers */

/** Enquiry ioctl.
 *  \return nfdev_enquiry_str describing the attached device.
 */
#define NFDEV_IOCTL_NUM_ENQUIRY        0x01

/** Channel Update ioctl.
 *  \deprecated
 */
#define NFDEV_IOCTL_NUM_CHUPDATE       0x02

/** Ensure Reading ioctl.
 *  Signal a read request to the device.
 *  \param (unsigned int) Length of data to be read.
 */
#define NFDEV_IOCTL_NUM_ENSUREREADING  0x03

/** Device Count ioctl.
 *  Not implemented for on all platforms.
 *  \return (int) the number of attached devices.
 */
#define NFDEV_IOCTL_NUM_DEVCOUNT       0x04

/** Internal Debug ioctl.
 *  Not implemented in release drivers.
 */
#define NFDEV_IOCTL_NUM_DEBUG          0x05

/** PCI Interface Version ioctl.
 *  \param (int) Maximum PCI interface version
 *   supported by the user of the device.
 */
#define NFDEV_IOCTL_NUM_PCI_IFVERS     0x06

/** Statistics ioctl.
 *  \return nfdev_enquiry_str describing the attached device.
 */
#define NFDEV_IOCTL_NUM_STATS          0x07

/** Module control ioctl
 * \param (nfdev_control_str) Value to write to HSM control register
 */
#define NFDEV_IOCTL_NUM_CONTROL        0x08

/** Module state ioctl
 * \return (nfdev_status_str) Values read from HSM status/error registers
 */
#define NFDEV_IOCTL_NUM_STATUS         0x09

#define NFDEV_IOCTL_TYPE 0x10

#define NFDEV_IOCTL_CHUPDATE                                                   \
	_IO(NFDEV_IOCTL_TYPE, NFDEV_IOCTL_NUM_CHUPDATE)

#define NFDEV_IOCTL_ENQUIRY                                                    \
	_IOR(NFDEV_IOCTL_TYPE, NFDEV_IOCTL_NUM_ENQUIRY,                        \
	     struct nfdev_enquiry_str)

#define NFDEV_IOCTL_ENSUREREADING                                              \
	_IOW(NFDEV_IOCTL_TYPE, NFDEV_IOCTL_NUM_ENSUREREADING, int)

#define NFDEV_IOCTL_ENSUREREADING_BUG3349                                      \
	_IO(NFDEV_IOCTL_TYPE, NFDEV_IOCTL_NUM_ENSUREREADING)

#define NFDEV_IOCTL_DEBUG                                                      \
	_IOW(NFDEV_IOCTL_TYPE, NFDEV_IOCTL_NUM_DEBUG, int)

#define NFDEV_IOCTL_PCI_IFVERS                                                 \
	_IOW(NFDEV_IOCTL_TYPE, NFDEV_IOCTL_NUM_PCI_IFVERS, int)

#define NFDEV_IOCTL_STATS                                                      \
	_IOR(NFDEV_IOCTL_TYPE, NFDEV_IOCTL_NUM_STATS, struct nfdev_stats_str)

#define NFDEV_IOCTL_CONTROL                                                    \
	_IOW(NFDEV_IOCTL_TYPE, NFDEV_IOCTL_NUM_CONTROL,                        \
	     const struct nfdev_control_str)

#define NFDEV_IOCTL_STATUS                                                     \
	_IOR(NFDEV_IOCTL_TYPE, NFDEV_IOCTL_NUM_STATUS, struct nfdev_status_str)

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
