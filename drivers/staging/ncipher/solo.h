/* SPDX-License-Identifier: GPL-2.0+ */
/** @file
 *
 * Declares the nFast PCI register interface, driver runtime
 * structures, and supporting items.
 *
 * The interface presented by nFast PCI devices consists of:
 *
 * - a region of shared RAM used for data transfer & control information
 * - a doorbell interrupt register, so both sides can give each other
 *   interrupts
 * - a number of DMA channels for transferring data
 *
 */

#ifndef SOLO_H
#define SOLO_H

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
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/nshield_solo.h>

/* Sizes of some regions */

/*
 * This is the minimum size of shared RAM. In future it may be possible to
 * negotiate larger sizes of shared RAM or auto-detect how big it is
 */
#define NFPCI_RAM_MINSIZE		    0x00100000
#define NFPCI_RAM_MINSIZE_JOBS		    0x00020000 /* standard jobs only */
#define NFPCI_RAM_MINSIZE_KERN		    0x00040000 /* standard and kernel
							* jobs
							*/

/* Offsets within shared memory space.
 * The following main regions are:
 *   jobs input area
 *   jobs output area
 *   kernel jobs input area
 *   kernel output area
 */

#define NFPCI_OFFSET_JOBS		    0x00000000
#define NFPCI_OFFSET_JOBS_WR		    0x00000000
#define NFPCI_OFFSET_JOBS_RD		    0x00010000
#define NFPCI_OFFSET_KERN		    0x00020000
#define NFPCI_OFFSET_KERN_WR		    0x00020000
#define NFPCI_OFFSET_KERN_RD		    0x00030000

/* Interrupts, defined by bit position in doorbell register */

/* Interrupts from device to host */
#define NFAST_INT_DEVICE_WRITE_OK	    0x00000001
#define NFAST_INT_DEVICE_WRITE_FAILED	    0x00000002
#define NFAST_INT_DEVICE_READ_OK	    0x00000004
#define NFAST_INT_DEVICE_READ_FAILED	    0x00000008
#define NFAST_INT_DEVICE_KERN_WRITE_OK	    0x00000010
#define NFAST_INT_DEVICE_KERN_WRITE_FAILED  0x00000020
#define NFAST_INT_DEVICE_KERN_READ_OK	    0x00000040
#define NFAST_INT_DEVICE_KERN_READ_FAILED   0x00000080

/* Interrupts from host to device */
#define NFAST_INT_HOST_WRITE_REQUEST	    0x00010000
#define NFAST_INT_HOST_READ_REQUEST	    0x00020000
#define NFAST_INT_HOST_DEBUG		    0x00040000
#define NFAST_INT_HOST_KERN_WRITE_REQUEST   0x00080000
#define NFAST_INT_HOST_KERN_READ_REQUEST    0x00100000

/* Ordinary job submission ------------------------ */

/*
 * The NFPCI_OFFSET_JOBS_WR and NFPCI_OFFSET_JOBS_RD regions
 * are defined by the following (byte) address offsets.
 */

#define NFPCI_OFFSET_CONTROL		    0x0
#define NFPCI_OFFSET_LENGTH		    0x4
#define NFPCI_OFFSET_DATA		    0x8
#define NFPCI_OFFSET_PUSH_ADDR		    0x8
#define NFPCI_OFFSET_PULL_ADDR		    0x8

#define NFPCI_JOBS_WR_CONTROL		    (NFPCI_OFFSET_JOBS_WR + \
					     NFPCI_OFFSET_CONTROL)
#define NFPCI_JOBS_WR_LENGTH		    (NFPCI_OFFSET_JOBS_WR + \
					     NFPCI_OFFSET_LENGTH)
#define NFPCI_JOBS_WR_DATA		    (NFPCI_OFFSET_JOBS_WR + \
					     NFPCI_OFFSET_DATA)
/* address in PCI space of host buffer for NFPCI_JOB_CONTROL_PCI_PUSH */
#define NFPCI_JOBS_WR_PULL_ADDR		    (NFPCI_OFFSET_JOBS_WR + \
					     NFPCI_OFFSET_PULL_ADDR)
#define NFPCI_MAX_JOBS_WR_LEN		    (0x0000FFF8)

#define NFPCI_JOBS_RD_CONTROL		    (NFPCI_OFFSET_JOBS_RD + \
					     NFPCI_OFFSET_CONTROL)
#define NFPCI_JOBS_RD_LENGTH		    (NFPCI_OFFSET_JOBS_RD + \
					     NFPCI_OFFSET_LENGTH)
#define NFPCI_JOBS_RD_DATA		    (NFPCI_OFFSET_JOBS_RD + \
					     NFPCI_OFFSET_DATA)
/* address in PCI space of host buffer for NFPCI_JOB_CONTROL_PCI_PUSH */
#define NFPCI_JOBS_RD_PUSH_ADDR		    (NFPCI_OFFSET_JOBS_RD + \
					     NFPCI_OFFSET_PUSH_ADDR)
#define NFPCI_MAX_JOBS_RD_LEN		    (0x000FFF8)

/* Kernel interface job submission ---------------- */

#define NFPCI_KERN_WR_CONTROL		    (NFPCI_OFFSET_KERN_WR + \
					     NFPCI_OFFSET_CONTROL)
#define NFPCI_KERN_WR_LENGTH		    (NFPCI_OFFSET_KERN_WR + \
					     NFPCI_OFFSET_LENGTH)
#define NFPCI_KERN_WR_DATA		    (NFPCI_OFFSET_KERN_WR + \
					     NFPCI_OFFSET_DATA)
/* address in PCI space of host buffer for NFPCI_JOB_CONTROL_PCI_PUSH */
#define NFPCI_KERN_WR_PULL_ADDR		    (NFPCI_OFFSET_KERN_WR + \
					     NFPCI_OFFSET_PULL_ADDR)
#define NFPCI_MAX_KERN_WR_LEN	            (0x0000FFF8)

#define NFPCI_KERN_RD_CONTROL		    (NFPCI_OFFSET_KERN_RD + \
					     NFPCI_OFFSET_CONTROL)
#define NFPCI_KERN_RD_LENGTH		    (NFPCI_OFFSET_KERN_RD + \
					     NFPCI_OFFSET_LENGTH)
#define NFPCI_KERN_RD_DATA		    (NFPCI_OFFSET_KERN_RD + \
					     NFPCI_OFFSET_DATA)
/* address in PCI space of host buffer for NFPCI_JOB_CONTROL_PCI_PUSH */
#define NFPCI_KERN_RD_PUSH_ADDR		    (NFPCI_OFFSET_KERN_RD + \
					     NFPCI_OFFSET_PUSH_ADDR)
#define NFPCI_MAX_KERN_RD_LEN		    (0x000FFF8)

#define NFPCI_JOB_CONTROL		    0x00000001
#define NFPCI_JOB_CONTROL_PCI_PUSH	    0x00000002
#define NFPCI_JOB_CONTROL_PCI_PULL	    0x00000003

/*
 * The 'Control' word is analogous to the SCSI read/write address;
 * 1 = standard push/pull I/O
 * 2 = push/push I/O
 * 3 = pull/push I/O
 *
 * To submit a block of job data, the host:
 * - sets the (32-bit, little-endian) word at NFPCI_JOBS_WR_CONTROL to
 * NFPCI_JOB_CONTROL
 * - sets the word at NFPCI_JOBS_WR_LENGTH to the length of the data
 * - copies the data to NFPCI_JOBS_WR_DATA
 * - sets interrupt NFAST_INT_HOST_WRITE_REQUEST in the doorbell register
 * - awaits the NFAST_INT_DEVICE_WRITE_OK (or _FAILED) interrupts back
 *
 * To read a block of jobs back, the host:
 * - sets the word at NFPCI_JOBS_RD_CONTROL to NFPCI_JOB_CONTROL
 * - sets the word at NFPCI_JOBS_RD_LENGTH to the max length for returned data
 * - sets interrupt NFAST_INT_HOST_READ_REQUEST
 * - awaits the NFAST_INT_DEVICE_READ_OK (or _FAILED) interrupt
 * - reads the data from NFPCI_JOBS_RD_DATA; the module will set the word at
 * NFPCI_JOBS_RD_LENGTH to its actual length.
 *
 * Optionally the host can request the PCI read data to be pushed to host PCI
 * mapped ram:
 * - allocates a contiguous PCI addressable buffer for a NFPCI_JOBS_BLOCK of
 * max size NFPCI_MAX_JOBS_RD_LEN (or NFPCI_MAX_KERN_RD_LEN) + 8
 * - sets the word at NFPCI_JOBS_RD_CONTROL to NFPCI_JOB_CONTROL_PCI_PUSH
 * - sets the word at NFPCI_JOBS_RD_LENGTH to the max length for returned data
 * - sets the word at NFPCI_JOBS_RD_PUSH_ADDR to be the host PCI address of
 * the buffer
 * - sets interrupt NFAST_INT_HOST_READ_REQUEST
 * - awaits the NFAST_INT_DEVICE_READ_OK (or _FAILED) interrupt
 * - reads the data from the buffer at NFPCI_OFFSET_DATA in the buffer. The
 * module will set NFPCI_OFFSET_LENGTH to the actual length.
 *
 * Optionally the host can request the PCI write data to be pulled from host
 * PCI mapped ram:
 * - allocates a contiguous PCI addressable buffer for a NFPCI_JOBS_BLOCK of
 * max size NFPCI_MAX_JOBS_WR_LEN (or NFPCI_MAX_KERN_WR_LEN) + 8
 * - copies the data to the PCI addressable buffer
 * - sets the word at NFPCI_JOBS_WR_CONTROL to NFPCI_JOB_CONTROL_PCI_PULL
 * - sets the word at NFPCI_JOBS_WR_LENGTH to the length of the data
 * - sets the word at NFPCI_JOBS_RD_PULL_ADDR to be the host PCI address of
 * the buffer
 * - sets interrupt NFAST_INT_HOST_WRITE_REQUEST in the doorbell register
 * - awaits the NFAST_INT_DEVICE_WRITE_OK (or _FAILED) interrupts back
 */

#define NFPCI_SCRATCH_CONTROL		    0

#define NFPCI_SCRATCH_CONTROL_HOST_MOI	    (0x1)
#define NFPCI_SCRATCH_CONTROL_MODE_SHIFT    1
#define NFPCI_SCRATCH_CONTROL_MODE_MASK	    (3 << \
					     NFPCI_SCRATCH_CONTROL_MODE_SHIFT)

#define NFPCI_SCRATCH_STATUS		    1

#define NFPCI_SCRATCH_STATUS_MONITOR_MOI         (0x1)
#define NFPCI_SCRATCH_STATUS_APPLICATION_MOI     (0x2)
#define NFPCI_SCRATCH_STATUS_APPLICATION_RUNNING (0x4)
#define NFPCI_SCRATCH_STATUS_ERROR               (0x8)

#define NFPCI_SCRATCH_ERROR_LO		    2
#define NFPCI_SCRATCH_ERROR_HI		    3

#define NFP_BARSIZES_COUNT 6
#define NFP_BARSIZES_MASK ~0xF

/* per-instance device structure ------------------------------------ */
struct nfp_dev {
	/* downward facing part of the device interface */
	u8 *bar[NFP_BARSIZES_COUNT];
	void *extra[NFP_BARSIZES_COUNT];
	int busno;
	int slotno;
	void *cmdctx;
	char *iobuf;
	int active_bar;
	int created;
	int conn_status;
	int detection_type;
	int iosize[6];
	u32 irq;
	struct nfpcmd_dev const *cmddev;
	struct nfdev_stats_str stats;

	/* upward facing part of the device interface */
	u8 *read_buf;
	dma_addr_t read_dma;

	u8 *write_buf;
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

	spinlock_t spinlock;     /* protect this struct */
	struct mutex ioctl_mutex;  /* lock across ioctl */
};

/* Per-device-type command handlers --------------------------------- */
struct nfpcmd_dev {
	const char *name;
	u16 vendorid, deviceid, sub_vendorid, sub_deviceid;
	u32 bar_sizes[NFP_BARSIZES_COUNT]; /* includes IO bit */
	u32 flags, max_ifvers;

	int (*create)(struct nfp_dev *ndev);
	int (*destroy)(void *ctx);
	int (*started)(struct nfp_dev *ndev, int lock_flag);
	int (*stopped)(struct nfp_dev *ndev);
	int (*open)(void *ctx);
	int (*close)(void *ctx);
	int (*isr)(void *ctx, int *handled);
	int (*write_block)(u32 addr, const char *ublock, int len, void *ctx);
	int (*read_block)(char *ublock, int len, void *ctx, int *rcount);
	int (*channel_update)(char *data, int len, void *ctx);
	int (*ensure_reading)(u32 addr, int len, void *ctx, int lock_flag);
	int (*debug)(int cmd, void *ctx);
	int (*setcontrol)(const struct nfdev_control_str *control,
			  void *ctx); /* may be NULL */
	int (*getstatus)(struct nfdev_status_str *status,
			 void *ctx); /* may be NULL */
};

/* These instances are defined in the per-board driver modules */
extern const struct nfpcmd_dev i21555_cmddev;
extern const struct nfpcmd_dev fsl_t1022_cmddev;

/* user and device memory space access ---------------------------- */
/*
 * NB these 2 functions are not guaranteed to be re-entrant for a given device
 */
int nfp_copy_from_user_to_dev(struct nfp_dev *ndev, int bar, int offset,
			      const char *ubuf, int len);
int nfp_copy_to_user_from_dev(struct nfp_dev *ndev, int bar, int offset,
			      char *ubuf, int len);

int nfp_copy_from_dev(struct nfp_dev *ndev, int bar, int offset,
		      char *kbuf, int len);
int nfp_copy_to_dev(struct nfp_dev *ndev, int bar, int offset,
		    const char *kbuf, int len);

#define NFP_CMD_FLG_NEED_MSI   0x2

#ifndef PCI_BASE_ADDRESS_SPACE_PREFETCHABLE
#define PCI_BASE_ADDRESS_SPACE_PREFETCHABLE 0x8
#endif

/*
 * This masks off the bottom bits of the PCI_CSR_BAR which signify that the
 * BAR is an IO BAR rather than a MEM BAR
 */
#define NFP_MEMBAR_MASK ~0xf

/* callbacks from command drivers -------------------------------------- */
void nfp_read_complete(struct nfp_dev *ndev, int ok);
void nfp_write_complete(struct nfp_dev *ndev, int ok);

/* debug ------------------------------------------------------------ */

#define NFP_DBG1	1
#define NFP_DBGE	NFP_DBG1
#define NFP_DBG2	2
#define NFP_DBG3	3
#define NFP_DBG4	4

void nfp_log(int severity, const char *format, ...);
extern int nfp_debug;

/* internal error codes --------------------------------------------- */
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

/* error conversions table */
struct errstr {
	int oserr;
	int nferr;
};

extern struct errstr errtab[];

#endif /* SOLO_H */
