// SPDX-License-Identifier: GPL-2.0+

/** @file
 *
 * hostif.c: nCipher PCI HSM linux host interface
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#ifdef MODVERSIONS
#include <linux/modversions.h>
#endif
#include "nfp_common.h"
#include "nfp_error.h"
#include "nfp_fixup.h"
#include "nfp_hostif.h"
#include "nfp_dev.h"
#include "nfp_osif.h"
#include "nfp_cmd.h"
#include "devinit.h"
#include <linux/bitops.h>

#include "i21555.h"
#include "fsl.h"
#include "fsl_osif.h"

#define NF_smp_mb__before_atomic smp_mb__before_atomic

/* device list --------------------------------------------------- */

struct nfp_dev *nfp_dev_list[NFP_MAXDEV];
static int nfp_num_devices;

static struct class *nfp_class;
/*! @} */

/**
 * @addtogroup modparams
 * Module structures.
 * @{
 */

/**
 * NSHIELD SOLO module debug parameter.
 *
 * This value can be overridden when the module is loaded, for example:
 *
 *   insmod nshield_solo.ko nfp_debug=<n>
 *
 * where <n> = 1 through 4 inclusive
 */
int nfp_debug = 1;

/**
 * NSHIELD_SOLO module interface version parameter.
 *
 * This value can be overridden when the module is loaded, for example:
 *
 *   insmod nshield_solo.ko nfp_ifvers=<n>
 *
 * where n = 0 allows any supported interface,
 *       n > 0 allows only interface versions <= n.
 * See nfdev-common.h for a list of supported interface versions.
 * Specific card models may not support all interface versions.
 */
int nfp_ifvers;

MODULE_AUTHOR("nCipher");
MODULE_DESCRIPTION("nCipher PCI HSM driver");
module_param(nfp_debug, int, 0644);
module_param(nfp_ifvers, int, 0444);
MODULE_PARM_DESC(nfp_debug, "debug level (1-4)");
MODULE_PARM_DESC(nfp_ifvers, "maximum interface version (1-2), or any (0)");

MODULE_LICENSE("GPL");

/** @} */

/**
 * @addtogroup fops
 * NSHIELD_SOLO character device file operations.
 * @{
 */

/**
 * Polls an NSHIELD SOLO device.
 *
 * The kernel calls this function when a user tries to poll an NSHIELD SOLO
 * device. The function returns a bit mask which indicates if the device is
 * immediately readable or writable.  A readable device will set
 * (POLLIN | POLLRDNORM). A writable device will set (POLLOUT | POLLWRNORM).
 *
 * @param filep device file pointer.
 * @param wait  poll table pointer.
 * @returns mask indicating if readable and/or writable.
 */
static unsigned int nfp_poll(struct file *file, poll_table *wait)
{
	struct nfp_dev *ndev;
	unsigned int mask = 0;
	int minor = MINOR(INODE_FROM_FILE(file)->i_rdev);

	nfp_log(NFP_DBG4, "%s: entered", __func__);

	if (minor >= NFP_MAXDEV) {
		nfp_log(NFP_DBG1, "%s: minor out of range.", __func__);
		return -ENODEV;
	}

	/* find ndev from minor */

	ndev = nfp_dev_list[minor];
	if (!ndev) {
		nfp_log(NFP_DBG1,
			"%s: no NSHIELD SOLO device associated with this file",
			__func__);
		return -ENODEV;
	}

	poll_wait(file, &ndev->wr_queue, wait);
	poll_wait(file, &ndev->rd_queue, wait);

	if (test_bit(0, &ndev->wr_ready))
		mask |= POLLOUT | POLLWRNORM; /* writeable */
	if (test_bit(0, &ndev->rd_ready))
		mask |= POLLIN | POLLRDNORM; /* readable */

	nfp_log(NFP_DBG3, "%s: device is %swritable, %sreadable",
		__func__,
		mask & POLLOUT ? "" : "not ", mask & POLLIN ? "" : "not ");

	return mask;
}

void nfp_write_complete(struct nfp_dev *ndev, int ok)
{
	nfp_log(NFP_DBG4, "%s: entered", __func__);

	/* could be executed simultaneously by more than one thread -
	 * e.g. from the write isr and from the nfp_write/timeout
	 * we don't want that to happen.
	 */
	if (test_and_set_bit(CMPLT_BIT, &ndev->wr_outstanding))
		return;

	if (!test_bit(WAIT_BIT, &ndev->wr_outstanding)) {
		/* we can only get here if the write has already
		 * been completed
		 */
		nfp_log(NFP_DBG1,
			"%s: no write outstanding to complete; ignoring completion",
			__func__);
		clear_bit(CMPLT_BIT, &ndev->wr_outstanding);
		return;
	}

	/* complete write by waking waiting processes */
	ndev->wr_ok = ok;

	nfp_log(NFP_DBG3, "%s: write completed %sokay",
		__func__,
		ok ? "" : "not ");

	NF_smp_mb__before_atomic();
	clear_bit(CMPLT_BIT, &ndev->wr_outstanding);
	clear_bit(WAIT_BIT, &ndev->wr_outstanding);

	/* wake up anyone waiting */

	wake_up_all(&ndev->wr_queue);
}

#define CREATE_TRACE_POINTS

/**
 * Writes to an NSHIELD SOLO device.
 *
 * Data in the user space buffer is written to the NSHIELD SOLO device. Any
 * previous data is overwritten. An error is returned if not all
 * bytes are written from the user space buffer.
 *
 * @param file  device file pointer.
 * @param buf   pointer to a user space buffer.
 * @param count size of user space buffer.
 * @param off   offset position (ignored).
 * @returns actual number of bytes written or a negative value if an erroR
 * occurred.
 */
static ssize_t nfp_write(struct file *file, char const __user *buf,
			 size_t count, loff_t *off)
{
	struct nfp_dev *ndev;
	unsigned int addr;
	int nbytes;
	int minor;
	int ne;
	(void)off;

	nfp_log(NFP_DBG4, "%s: entered", __func__);

	/* find ndev from minor */

	minor = MINOR(INODE_FROM_FILE(file)->i_rdev);
	if (minor >= NFP_MAXDEV) {
		nfp_log(NFP_DBG1, "%s: minor out of range.", __func__);
		return -ENODEV;
	}

	ndev = nfp_dev_list[minor];
	if (!ndev) {
		nfp_log(NFP_DBG1, "%s: write: NULL ndev.", __func__);
		return -ENODEV;
	}

	/* check max length requested */

	if (count <= 0 || NFP_WRITE_MAX < count) {
		nfp_log(NFP_DBG1,
			"%s: invalid requested write length %lu",
			__func__, count);
		return -EINVAL;
	}

	/* check if called before ready */

	if (!test_and_clear_bit(0, &ndev->wr_ready)) {
		nfp_log(NFP_DBG1, "%s: write called when not ready.",
			__func__);
		return -ENXIO;
	}
	set_bit(WAIT_BIT, &ndev->wr_outstanding);

	nfp_log(NFP_DBG3, "%s: writing %d bytes", __func__, count);

	addr = 0;
	if (ndev->ifvers >= NFDEV_IF_PCI_PULL) {
		nfp_log(NFP_DBG3, "%s: copying %lu bytes to dma buffer",
			__func__, count);
		addr = ndev->write_dma;
		TO_LE32_MEM(&nbytes, count);
		*(unsigned int *)(ndev->write_buf + NFP_DMA_NBYTES_OFFSET) =
			nbytes;
		if (0 !=
		    copy_from_user(ndev->write_buf + NFP_DMA_ADDRESS_OFFSET,
				   buf, count)) {
			clear_bit(WAIT_BIT, &ndev->wr_outstanding);
			set_bit(0, &ndev->wr_ready);
			nfp_log(NFP_DBG1,
				"%s: copy from user space failed", __func__);
			return -EIO;
		}
	}

	ne = ndev->cmddev->write_block(addr, buf, count, ndev->common.cmdctx);
	if (ne != NFP_SUCCESS) {
		nfp_write_complete(ndev, 0);
		if (ne != NFP_ESTARTING)
			nfp_log(NFP_DBG1, "%s: write_block failed", __func__);
	}

	while (test_bit(WAIT_BIT, &ndev->wr_outstanding)) {
		if (!wait_event_timeout(ndev->wr_queue,
					test_bit(WAIT_BIT,
						 &ndev->wr_outstanding) == 0,
					NFP_TIMEOUT)) {
			nfp_write_complete(ndev, 0);
			set_bit(0, &ndev->wr_ready);
			nfp_log(NFP_DBG1, "%s: module timed out", __func__);
			return -ENXIO;
		}
		if (test_bit(WAIT_BIT, &ndev->wr_outstanding)) {
			nfp_log(NFP_DBG1,
				"%s: handling spurious wake-up", __func__);
		}
	}
	set_bit(0, &ndev->wr_ready);

	nfp_log(NFP_DBG2, "%s: returning %d.", __func__,
		ndev->wr_ok ? count : -EIO);

	if (!ndev->wr_ok) {
		nfp_log(NFP_DBG1, "%s: device write failed", __func__);
		return -EIO;
	}

	nfp_log(NFP_DBG2, "%s: wrote %d bytes (%s)", __func__, count,
		ndev->ifvers >= NFDEV_IF_PCI_PULL ? "dma" : "std");
	return count;
}

void nfp_read_complete(struct nfp_dev *ndev, int ok)
{
	nfp_log(NFP_DBG4, "%s: entered", __func__);

	/* could be executed simultaneously by more than one thread -
	 * e.g. from the read isr and from the timeout
	 * we don't want that to happen.
	 */
	if (test_and_set_bit(CMPLT_BIT, &ndev->rd_outstanding))
		return;

	if (!test_bit(WAIT_BIT, &ndev->rd_outstanding)) {
		/* we can only get here if the read has already been completed
		 * and no new ENSUREREADING request has been received since
		 */
		nfp_log(NFP_DBG1,
			"%s: no read outstanding to complete; ignoring completion",
			__func__);
		clear_bit(CMPLT_BIT, &ndev->rd_outstanding);
		return;
	}

	/* in case the timer has not expired */
	del_timer(&ndev->rd_timer);

	ndev->rd_ok = ok;
	set_bit(0, &ndev->rd_ready);

	nfp_log(NFP_DBG3, "%s: read completed %sokay",
		__func__,
		ok ? "" : "not ");

	NF_smp_mb__before_atomic();
	clear_bit(CMPLT_BIT, &ndev->rd_outstanding);
	clear_bit(WAIT_BIT, &ndev->rd_outstanding);

	/* wake up anyone waiting */

	wake_up_all(&ndev->rd_queue);
}

static void nfp_read_timeout(struct timer_list *t)
{
	struct nfp_dev *ndev;

	nfp_log(NFP_DBG4, "%s: entered",
		__func__);

	ndev = from_timer(ndev, t, rd_timer);

	if (!ndev) {
		nfp_log(NFP_DBG1, "%s: NULL device.", __func__);
		return;
	}

	nfp_read_complete(ndev, 0);
}

/**
 * Reads from an NSHIELD SOLO device.
 *
 * Data is read from the NSHIELD SOLO device into the user space buffer.
 * The read removes the data from the device. An error is returned is not
 * all the bytes are read from the user space buffer.
 *
 * @param file  device file pointer.
 * @param buf   pointer to a user space buffer.
 * @param count maximum size of user space buffer.
 * @param off   offset position (ignored).
 * @returns actual number of bytes read or a negative
 * value if an error occurred.
 */
static ssize_t nfp_read(struct file *file, char __user *buf, size_t count,
			loff_t *off)
{
	struct nfp_dev *ndev;
	int nbytes;
	int minor;
	int ne;
	(void)off;

	nfp_log(NFP_DBG4, "%s: entered", __func__);

	minor = MINOR(INODE_FROM_FILE(file)->i_rdev);
	if (minor >= NFP_MAXDEV) {
		nfp_log(NFP_DBG1, "%s: minor out of range.", __func__);
		return -ENODEV;
	}

	ndev = nfp_dev_list[minor];
	if (!ndev) {
		nfp_log(NFP_DBG1, "%s: NULL ndev.", __func__);
		return -ENODEV;
	}

	/* check user space buffer */

	if (!access_ok(buf, count)) {
		nfp_log(NFP_DBG1, "%s: user space verify failed.", __func__);
		return -EFAULT;
	}

	/* check max length requested */

	if (count <= 0 || NFP_READ_MAX < count) {
		nfp_log(NFP_DBG1,
			"%s: invalid requested max read length %lu",
			__func__,
			count);
		return -EINVAL;
	}

	if (!test_and_clear_bit(0, &ndev->rd_ready)) {
		nfp_log(NFP_DBG1,
			"%s: read called when not ready.",
			__func__);
		return -EIO;
	}
	nbytes = 0;

	/* check if read was ok */

	if (!ndev->rd_ok) {
		nfp_log(NFP_DBG1, "%s: read failed", __func__);
		return -EIO;
	}

	/* finish read */

	if (ndev->ifvers >= NFDEV_IF_PCI_PUSH) {
		nbytes = *(unsigned int *)(ndev->read_buf +
					   NFP_DMA_NBYTES_OFFSET);
		nbytes = FROM_LE32_MEM(&nbytes);
		nfp_log(NFP_DBG3, "%s: nbytes %d", __func__, nbytes);
		if (nbytes < 0 || nbytes > count) {
			nfp_log(NFP_DBG1,
				"%s: bad byte count (%d) from device",
				__func__, nbytes);
			return -EIO;
		}
		if (nfp_copy_to_user(buf,
				     ndev->read_buf + NFP_DMA_ADDRESS_OFFSET,
				     nbytes) != 0) {
			nfp_log(NFP_DBG1,
				"%s: copy to user space failed", __func__);
			return -EIO;
		}
	} else {
		nbytes = 0;
		ne = ndev->cmddev->read_block(buf, count, ndev->common.cmdctx,
					      (void *)&nbytes);
		if (ne != NFP_SUCCESS) {
			nfp_log(NFP_DBG1, "%s: device read failed", __func__);
			return nfp_oserr(ne);
		}
	}

	if (nbytes > NFP_READ_MAX) {
		nfp_log(NFP_DBG1, "%s: read reply overflow: %d > %d max",
			nbytes, NFP_READ_MAX);
		return -EIO;
	}

	nfp_log(NFP_DBG2, "%s: read %d bytes (%s)", __func__, nbytes,
		ndev->ifvers >= NFDEV_IF_PCI_PUSH ? "dma" : "std");
	return nbytes;
}

int nfp_alloc_pci_push(struct nfp_dev *ndev)
{
	/* allocate resources needed for PCI Push,
	 * if not already allocated.
	 * return True if successful
	 */
	if (!ndev->read_buf) {
		ndev->read_buf =
			kmalloc(NFP_READBUF_SIZE, GFP_KERNEL | GFP_DMA);
		if (ndev->read_buf) {
			memset(ndev->read_buf, 0, NFP_READBUF_SIZE);
			ndev->read_dma =
				dma_map_single(&ndev->pcidev->dev,
					       ndev->read_buf, NFP_READBUF_SIZE,
					       DMA_BIDIRECTIONAL);
			if (dma_mapping_error(&ndev->pcidev->dev,
					      ndev->read_dma)) {
				nfp_log(NFP_DBG1,
					"dma_mapping_error found after attempting dma_map_single");
				kfree(ndev->read_buf);
				ndev->read_buf = NULL;
				ndev->read_dma = 0;
			}
		}
	}
	return ndev->read_buf ? 1 : 0;
}

int nfp_alloc_pci_pull(struct nfp_dev *ndev)
{
	/* allocate resources needed for PCI Pull,
	 * if not already allocated.
	 * return True if successful
	 */
	if (!ndev->write_buf) {
		ndev->write_buf =
			kmalloc(NFP_WRITEBUF_SIZE, GFP_KERNEL | GFP_DMA);
		if (ndev->write_buf) {
			memset(ndev->write_buf, 0, NFP_WRITEBUF_SIZE);
			ndev->write_dma = dma_map_single(&ndev->pcidev->dev,
							 ndev->write_buf,
							 NFP_WRITEBUF_SIZE,
							 DMA_BIDIRECTIONAL);
			if (dma_mapping_error(&ndev->pcidev->dev,
					      ndev->write_dma)) {
				nfp_log(NFP_DBG1,
					"dma_mapping_error found after attempting dma_map_single");
				kfree(ndev->write_buf);
				ndev->write_buf = NULL;
				ndev->write_dma = 0;
			}
		}
	}
	return ndev->write_buf ? 1 : 0;
}

void nfp_free_pci_push(struct nfp_dev *ndev)
{
	/* free resources allocated to PCI Push */
	if (ndev->read_buf) {
		dma_unmap_single(&ndev->pcidev->dev, ndev->read_dma,
				 NFP_READBUF_SIZE, DMA_BIDIRECTIONAL);
		kfree(ndev->read_buf);
		ndev->read_buf = NULL;
		ndev->read_dma = 0;
	}
}

void nfp_free_pci_pull(struct nfp_dev *ndev)
{
	/* free resources allocated to PCI Pull */
	if (ndev->write_buf) {
		dma_unmap_single(&ndev->pcidev->dev, ndev->write_dma,
				 NFP_WRITEBUF_SIZE, DMA_BIDIRECTIONAL);
		kfree(ndev->write_buf);
		ndev->write_buf = NULL;
		ndev->write_dma = 0;
	}
}

/*
 * Sets device interface version.
 *
 * @param ndev an NSHIELD SOLO device.
 * @param ifvers interface version.
 * @returns interface version actually set.
 */
static int nfp_set_ifvers(struct nfp_dev *ndev, int ifvers)
{
#ifdef _WIN32
#pragma warning(disable : 6239)
#pragma warning(disable : 6326)
#endif
	int max_ifvers;

	nfp_log(NFP_DBG4, "%s: entered", __func__);

	max_ifvers = ndev->cmddev->max_ifvers;
	if (nfp_ifvers != 0 && max_ifvers > nfp_ifvers)
		max_ifvers = nfp_ifvers;

	/* on any error, ifvers remains unchanged */
	if (ifvers < 0 || ifvers > max_ifvers) {
		/* invalid nfp_ifvers: set to max as fallback */
		nfp_log(NFP_DBG1,
			"%s: %d out of allowable range [0:%d]",
			__func__,
			ifvers, max_ifvers);
		return ndev->ifvers;
	}

	if (ifvers == 0 &&
	    ndev->cmddev->deviceid == PCI_DEVICE_ID_FREESCALE_T1022) {
		/* default ifvers: set to max for ngsolo not for legacy!
		 * The legacy card needs it set to 0 when in Maintenance
		 * mode and then the hardserver steps it up to ifvers 2
		 * when switching back to Operational mode. Solo XC starts
		 * at the max (3) whenever possible.
		 */
		ifvers = max_ifvers;
	}

	if (ifvers >= NFDEV_IF_PCI_PUSH) {
		if (!nfp_alloc_pci_push(ndev)) {
			nfp_log(NFP_DBG1,
				"%s: can't set ifvers %d as resources not available",
				__func__,
				ifvers);
			return ndev->ifvers;
		}
	} else {
		nfp_free_pci_push(ndev);
	}

	if (ifvers >= NFDEV_IF_PCI_PULL) {
		if (!nfp_alloc_pci_pull(ndev)) {
			nfp_log(NFP_DBG1,
				"%s: can't set ifvers %d as resources not available",
				__func__,
				ifvers);
			return ndev->ifvers;
		}
	} else {
		nfp_free_pci_pull(ndev);
	}

	ndev->ifvers = ifvers;
	nfp_log(NFP_DBG2, "%s: setting ifvers = %d", __func__, ifvers);

	return ifvers;
}

/**
 * Performs an NSHIELD SOLO device IOCTL call.
 *
 * @param inode device inode pointer.
 * @param file  device file pointer.
 * @param cmd   command id.
 * @param arg   command argument.
 * @returns 0 if successful.
 */
static int nfp_ioctl(struct inode *inode,
		     struct file *file,
		     unsigned int cmd,
		     unsigned long arg)
{
	struct nfp_dev *ndev;
	int minor;

	nfp_log(NFP_DBG4, "%s: entered", __func__);

	/* find ndev from minor */

	minor = MINOR(INODE_FROM_FILE(file)->i_rdev);

	if (minor >= NFP_MAXDEV) {
		nfp_log(NFP_DBG1, "%s: minor out of range.", __func__);
		return -ENODEV;
	}

	ndev = nfp_dev_list[minor];
	if (!ndev) {
		nfp_log(NFP_DBG1, "%s: NULL ndev.", __func__);
		return -ENODEV;
	}

	switch (cmd) {
	case NFDEV_IOCTL_ENQUIRY: {
		struct nfdev_enquiry_str enq_data;
		int err = -EIO;

		nfp_log(NFP_DBG4, "%s: enquiry", __func__);
		enq_data.busno = ndev->common.busno;
		enq_data.slotno = ndev->common.slotno;
		if ((void *)arg) {
			COPY_TO_USER((void *)arg, &enq_data, sizeof(enq_data),
				     err);
			if (err) {
				nfp_log(NFP_DBG1,
					"%s: copy to user space failed.",
					__func__);
				return err;
			}
		} else {
			nfp_log(NFP_DBG1,
				"%s enquiry: arg pointer is NULL!", __func__);
			return err;
		}
	} break;
	case NFDEV_IOCTL_ENSUREREADING:
	case NFDEV_IOCTL_ENSUREREADING_BUG3349: {
		unsigned int addr, len;
		int err = -EIO;
		int ne;

		nfp_log(NFP_DBG4, "%s: ensure reading", __func__);

		/* get and check max length */
		if ((void *)arg) {
			COPY_FROM_USER((void *)&len, (void *)arg,
				       sizeof(unsigned int), err);
			if (err) {
				nfp_log(NFP_DBG1,
					"%s: ensure reading: copy from user space failed.",
					__func__);
				return err;
			}
			/* signal a read to the module */
			nfp_log(NFP_DBG2,
				"%s: signalling read request to module, len = %x.",
				__func__,
				len);
			if (len > NFP_READ_MAX) {
				nfp_log(NFP_DBG1, "%s: len > %x = %x.",
					__func__,
					NFP_READ_MAX, len);
				return -EINVAL;
			}
		} else {
			nfp_log(NFP_DBG1,
				"%s ensure reading: arg pointer is NULL!",
				__func__);
			return err;
		}

		if (len <= 0 || NFP_READ_MAX < len) {
			nfp_log(NFP_DBG1,
				"%s: ensure reading: invalid max length %d/%d",
				__func__,
				len, NFP_READ_MAX);
			err = -EINVAL;
			return err;
		}

		/* check if okay to start read */

		if (test_and_set_bit(WAIT_BIT, &ndev->rd_outstanding)) {
			nfp_log(NFP_DBG1,
				"%s: ensure reading: another read is outstanding",
				__func__);
			return -EBUSY;
		}

		nfp_log(NFP_DBG2, "%s: ndev->rd_outstanding=1", __func__);

		/* start read ready timeout */

		mod_timer(&ndev->rd_timer, jiffies + (NFP_TIMEOUT_SEC * HZ));

		nfp_log(NFP_DBG2, "%s: read request", __func__);
		/* start read */

		addr = (ndev->ifvers < NFDEV_IF_PCI_PUSH) ? 0 : ndev->read_dma;
		nfp_log(NFP_DBG3,
			"%s: ensure reading: read request with ifvers=%d addr=%p",
			__func__,
			ndev->ifvers, addr);

		ne = ndev->cmddev->ensure_reading(addr, len,
						  ndev->common.cmdctx, 1);

		if (ne != NFP_SUCCESS) {
			nfp_log(NFP_DBG1,
				"%s: ensure reading: read request failed",
				__func__);
			del_timer_sync(&ndev->rd_timer);
			/* make sure that del_timer_sync is done before
			 * we clear rd_outstanding
			 */
			NF_smp_mb__before_atomic();
			clear_bit(WAIT_BIT, &ndev->rd_outstanding);
			return -EIO;
		}
	} break;

	case NFDEV_IOCTL_PCI_IFVERS: {
		int vers, err = -EIO;

		nfp_log(NFP_DBG4, "%s: set ifvers", __func__);
		if ((void *)arg) {
			COPY_FROM_USER(&vers, (void *)arg, sizeof(vers), err);
			if (err) {
				nfp_log(NFP_DBG1,
					"%s: set ifvers: copy from user space failed.",
					__func__);
				return err;
			}
		} else {
			nfp_log(NFP_DBG1,
				"%s ifvers: arg pointer is NULL!",
				__func__);
			return err;
		}

		if (test_bit(WAIT_BIT, &ndev->rd_outstanding)) {
			nfp_log(NFP_DBG1,
				"%s: set ifvers: unable to set interface version while read outstanding",
				__func__);
			return -EIO;
		}

		nfp_set_ifvers(ndev, vers);
	} break;

	case NFDEV_IOCTL_CHUPDATE:
		nfp_log(NFP_DBG1, "%s: channel update ignored", __func__);
		break;

	case NFDEV_IOCTL_DEBUG: {
		int num, err = -EIO;

		nfp_log(NFP_DBG4, "%s: debug", __func__);
		if ((void *)arg) {
			COPY_FROM_USER(&num, (void *)arg, sizeof(num), err);
			if (err) {
				nfp_log(NFP_DBG1,
					"%s: debug: copy from user space failed.",
					__func__);
				return err;
			}
		} else {
			nfp_log(NFP_DBG1,
				"%s debug: arg pointer is NULL!", __func__);
			return err;
		}
		if (ndev->cmddev->debug)
			return nfp_oserr(ndev->cmddev->debug(num,
					 ndev->common.cmdctx));

		return -EINVAL;
	} break;

	case NFDEV_IOCTL_STATS: {
		int err = -EIO;

		nfp_log(NFP_DBG4, "%s: stats", __func__);
		if ((void *)arg) {
			COPY_TO_USER((void *)arg, &ndev->common.stats,
				     sizeof(struct nfdev_stats_str), err);
			if (err) {
				nfp_log(NFP_DBG1,
					"%s: stats: copy to user space failed.",
					__func__);
				return err;
			}
		} else {
			nfp_log(NFP_DBG1,
				"%s stats: arg pointer is NULL!", __func__);
			return err;
		}
	} break;

	case NFDEV_IOCTL_CONTROL: {
		int err = -EIO;
		struct nfdev_control_str control;

		nfp_log(NFP_DBG4, "%s: control", __func__);
		if ((void *)arg) {
			COPY_FROM_USER(&control, (void *)arg, sizeof(control),
				       err);
			if (err) {
				nfp_log(NFP_DBG1,
					"%s: control: copy from user space failed.",
					__func__);
				return err;
			}
		} else {
			nfp_log(NFP_DBG1,
				"%s control: arg pointer is NULL!", __func__);
			return err;
		}
		if (!ndev->cmddev->setcontrol) {
			nfp_log(NFP_DBG2,
				"%s: control: set control not supported for this device: ignored",
				__func__);
			return -EINVAL;
		}
		nfp_log(NFP_DBG3,
			"%s: control: updating HSM control register to 0x%x.",
			__func__,
			control);

		return nfp_oserr(ndev->cmddev->setcontrol(&control,
							  ndev->common.cmdctx));
	} break;

	case NFDEV_IOCTL_STATUS: {
		int err = -EIO;
		struct nfdev_status_str status;

		nfp_log(NFP_DBG4, "%s: status", __func__);

		if (!ndev->cmddev->getstatus) {
			nfp_log(NFP_DBG2,
				"%s: status not supported for this device: ignored",
				__func__);
			return -EINVAL;
		}
		err = ndev->cmddev->getstatus(&status,
					      ndev->common.cmdctx);

		if (err)
			return nfp_oserr(err);

		if ((void *)arg) {
			COPY_TO_USER((void *)arg, &status, sizeof(status), err);
			if (err) {
				nfp_log(NFP_DBG1,
					"%s: status: copy from user space failed.",
					__func__);
				return err;
			}
		} else {
			nfp_log(NFP_DBG1,
				"%s status: arg pointer is NULL!",
				__func__);
			return err;
		}
		nfp_log(NFP_DBG3,
			"%s: read status: 0x%x, error: 0x%02x%02x%02x%02x%02x%02x%02x%02x",
			__func__,
			status.status, status.error[0], status.error[1],
			status.error[2], status.error[3], status.error[4],
			status.error[5], status.error[6], status.error[7]);
	} break;

	default: {
		nfp_log(NFP_DBG1, "%s: unknown ioctl.", __func__);
		return -EINVAL;
	} break;
	}

	return 0;
}

/**
 * Performs an NSHIELD SOLO device unlocked IOCTL call.
 *
 * @param file  device file pointer.
 * @param cmd   command id.
 * @param arg   command argument.
 * @returns 0 if successful.
 */
static long nfp_unlocked_ioctl(struct file *file,
			       unsigned int cmd,
			       unsigned long arg)
{
	long ret;
	int minor;
	struct nfp_dev *ndev;

	nfp_log(NFP_DBG2, "%s: entered", __func__);

	minor = MINOR(INODE_FROM_FILE(file)->i_rdev);
	ndev = nfp_dev_list[minor];
	if (!ndev) {
		nfp_log(NFP_DBG1, "%s: NULL ndev.", __func__);
		return -ENODEV;
	}
	spin_lock(&ndev->spinlock);

	ret = nfp_ioctl(NULL, file, cmd, arg);

	spin_unlock(&ndev->spinlock);
	nfp_log(NFP_DBG2, "%s: left", __func__);
	return ret;
}

static irqreturn_t nfp_isr(int irq, void *context)
{
	struct nfp_dev *ndev;
	int handled;
	int ne;

	nfp_log(NFP_DBG4, "%s: entered", __func__);

	ndev = (struct nfp_dev *)context;
	if (!ndev) {
		nfp_log(NFP_DBG1,
			"%s: no device associated with this interrupt",
			__func__);
		return IRQ_NONE;
	}

	ne = ndev->cmddev->isr(ndev->common.cmdctx, &handled);

	if (ne)
		nfp_log(NFP_DBG1, "%s: cmddev isr failed (%d)",
			__func__,
			nfp_oserr(ne));

	return IRQ_RETVAL(handled);
}

/**
 * Opens an NSHIELD SOLO device.
 *
 * The kernel calls this function when a user tries to open an
 * NSHIELD SOLO device.  It is an error to attempt to open an
 * already opened device.
 *
 * @param inode device inode pointer.
 * @param file device file pointer.
 * @return 0 if successful.
 */
static int nfp_open(struct inode *inode, struct file *file)
{
	struct nfp_dev *ndev;
	int ne;
	int minor = MINOR(INODE_FROM_FILE(file)->i_rdev);

	nfp_log(NFP_DBG4, "%s: entered", __func__);

	nfp_log(NFP_DBG3, "%s: opening file at %p.", __func__, file);

	/* find ndev */

	if (minor >= NFP_MAXDEV) {
		nfp_log(NFP_DBG1, "%s: minor out of range.", __func__);
		return -ENODEV;
	}

	ndev = nfp_dev_list[minor];
	if (!ndev) {
		nfp_log(NFP_DBG1, "%s: cannot find dev %d.", __func__, minor);
		return -ENODEV;
	}

	/* check if alreayd open */

	spin_lock(&ndev->spinlock);
	if (ndev->busy) {
		nfp_log(NFP_DBG1, "%s: device %s busy", __func__,
			pci_name(ndev->pcidev));
		spin_unlock(&ndev->spinlock);
		return -EBUSY;
	}
	ndev->busy = 1;
	spin_unlock(&ndev->spinlock);

	/* drop any old data */

	clear_bit(0, &ndev->rd_ready);

	/* set interface to module default */

	if (ndev->cmddev->deviceid == PCI_DEVICE_ID_FREESCALE_T1022)
		nfp_set_ifvers(ndev, NFDEV_IF_PCI_PULL);
	else
		nfp_set_ifvers(ndev, NFDEV_IF_STANDARD);

	nfp_log(NFP_DBG3, "%s: ifvers set to %d", __func__, ndev->ifvers);

	/* open device */

	ne = ndev->cmddev->open(ndev->common.cmdctx);
	if (ne != NFP_SUCCESS) {
		nfp_log(NFP_DBG1, "%s: device open failed: error %d",
			__func__,
			nfp_oserr(ne));
		spin_lock(&ndev->spinlock);
		ndev->busy = 0;
		spin_unlock(&ndev->spinlock);
		return nfp_oserr(ne);
	}

	nfp_log(NFP_DBG2, "%s: device %s open",
		__func__, pci_name(ndev->pcidev));

	return 0;
}

/**
 * Releases an NSHIELD SOLO device.
 *
 * The kernel calls this function when a user tries to close an
 * NSHIELD SOLO device.  It is an error to attempt to close an
 * already closed device.
 *
 * @param node device inode pointer.
 * @param file  device file pointer.
 * @returns 0 if successful.
 */
static int nfp_release(struct inode *node, struct file *file)
{
	struct nfp_dev *ndev;
	long timeout;
	int ne;
	int minor = MINOR(INODE_FROM_FILE(file)->i_rdev);

	nfp_log(NFP_DBG4, "%s: entered", __func__);

	nfp_log(NFP_DBG2, "%s: closing file at %p.", __func__, file);

	/* find ndev from minor */

	if (minor >= NFP_MAXDEV) {
		nfp_log(NFP_DBG1, "%s: minor out of range.", __func__);
		return(-ENODEV);
	}

	ndev = nfp_dev_list[minor];
	if (!ndev) {
		nfp_log(NFP_DBG1, "%s: cannot find dev.", __func__);
		return(-ENODEV);
	}

	{
		wait_queue_entry_t wait;

		timeout = 1;
		init_waitqueue_entry(&wait, current);
		current->state = TASK_UNINTERRUPTIBLE;
		add_wait_queue(&ndev->rd_queue, &wait);
		if (test_bit(WAIT_BIT, &ndev->rd_outstanding)) {
			nfp_log(NFP_DBG2, "%s: read outstanding", __func__);
			timeout = schedule_timeout(NFP_TIMEOUT);
			nfp_log(NFP_DBG3, "%s: finished waiting", __func__);
		}
		current->state = TASK_RUNNING;
		remove_wait_queue(&ndev->rd_queue, &wait);
		if (!timeout) {
			nfp_log(NFP_DBG1,
				"%s: outstanding read timed out", __func__);
		}
	}

	spin_lock(&ndev->spinlock);
	if (test_bit(WAIT_BIT, &ndev->rd_outstanding)) {
		del_timer_sync(&ndev->rd_timer);
		/* make sure that del_timer_sync is done before
		 * we clear rd_outstanding
		 */
		NF_smp_mb__before_atomic();
		clear_bit(WAIT_BIT, &ndev->rd_outstanding);
	}
	ndev->busy = 0;
	spin_unlock(&ndev->spinlock);

	/* close device */

	ne = ndev->cmddev->close(ndev->common.cmdctx);
	if (ne != NFP_SUCCESS) {
		nfp_log(NFP_DBG1, "%s: device close failed", __func__);
		return nfp_oserr(ne);
	}

	return(0);
}

/**
 * NSHIELD SOLO character device file operations table.
 */
static const struct file_operations nfp_fops = {
owner: THIS_MODULE,
poll : nfp_poll,
write : nfp_write,
read : nfp_read,
unlocked_ioctl : nfp_unlocked_ioctl,
open : nfp_open,
release : nfp_release,
};

/**
 * @addtogroup devmgr
 * NSHIELD SOLO device management.
 * @{
 */

/* device setup -------------------------------------------------- */

static void nfp_dev_destroy(struct nfp_dev *ndev, struct pci_dev *pci_dev)
{
	int i;

	nfp_log(NFP_DBG2, "%s: entered", __func__);
	if (ndev) {
		nfp_free_pci_push(ndev);
		nfp_free_pci_pull(ndev);

		if (ndev->irq) {
			nfp_log(NFP_DBG3, "%s: freeing irq, %x",
				__func__,
				ndev->irq);
			free_irq(ndev->irq, ndev);
		}
		for (i = 0; i < 6; i++)
			if (ndev->common.bar[i]) {
				nfp_log(NFP_DBG3,
					"%s: freeing MEM BAR, %d",
					__func__, i);
				release_mem_region(pci_resource_start(pci_dev,
								      i),
						   pci_resource_len(pci_dev,
								    i));
				iounmap(ndev->common.bar[i]);
			}
		nfp_log(NFP_DBG3, "%s: freeing ndev", __func__);
		kfree(ndev);
	}
}

static int nfp_setup(const struct nfpcmd_dev *cmddev, unsigned char bus,
		     unsigned char slot, unsigned int bar[6],
		     unsigned int irq_line, struct pci_dev *pcidev)
{
	struct nfp_dev *ndev = 0;
	int ne;
	int i;

	nfp_log(NFP_DBG2, "%s: Found '%s' at bus %x, slot %x, irq %d.",
		__func__,
		cmddev->name, bus, slot, irq_line);

	if (nfp_num_devices >= NFP_MAXDEV) {
		nfp_log(NFP_DBG1, "%s: minor out of range.", __func__);
		goto fail_continue;
	}

	ndev = kmalloc(sizeof(*ndev), GFP_KERNEL);
	if (!ndev) {
		nfp_log(NFP_DBG1,
			"%s: failed to allocate device structure.", __func__);
		goto fail_continue;
	}
	nfp_log(NFP_DBG2, "%s: allocated device structure.", __func__);
	memset(ndev, 0, sizeof(*ndev));

	ndev->common.busno = bus;
	ndev->pcidev = pcidev;
	ndev->common.slotno = slot;
	ndev->cmddev = cmddev;

	for (i = 0; i < 6; i++) {
		int map_bar_size = cmddev->bar_sizes[i] & NFP_MEMBAR_MASK;
		int bar_flags = cmddev->bar_sizes[i] & ~NFP_MEMBAR_MASK;

		if (map_bar_size) {
			if (!request_mem_region(pci_resource_start(pcidev, i),
						pci_resource_len(pcidev, i),
						cmddev->name)) {
				nfp_log(NFP_DBG1,
					"%s: request_mem_region failed, %x %x %d (%s)",
					__func__,
					pci_resource_start(pcidev, i),
					pci_resource_len(pcidev, i), i,
					cmddev->name);
				goto fail_continue;
			}

			if (bar_flags & PCI_BASE_ADDRESS_SPACE_PREFETCHABLE) {
				ndev->common.bar[i] =
					ioremap(bar[i], map_bar_size);
			} else {
				ndev->common.bar[i] =
					ioremap_nocache(bar[i], map_bar_size);
			}

			if (!ndev->common.bar[i]) {
				nfp_log(NFP_DBG1,
					"%s: unable to map memory BAR %d, (0x%x).",
					__func__,
					i, bar[i]);
				goto fail_continue;
			}
		}
	}

	init_waitqueue_head(&ndev->wr_queue);
	init_waitqueue_head(&ndev->rd_queue);

	set_bit(0, &ndev->wr_ready);

	ndev->spinlock = __SPIN_LOCK_UNLOCKED(ndev->spinlock);

	ne = ndev->cmddev->create(&ndev->common);
	if (ne != NFP_SUCCESS) {
		nfp_log(NFP_DBG1,
			"%s: failed to create command device (%d)", __func__,
			nfp_oserr(ne));
		goto fail_continue;
	}
	ndev->common.dev = ndev;

	if (request_irq(irq_line, nfp_isr, IRQF_SHARED, cmddev->name, ndev)) {
		nfp_log(NFP_DBG1, "%s: unable to claim interrupt.", __func__);
		goto fail_continue;
	}
	ndev->irq = irq_line;

	memset(&ndev->common.stats, 0, sizeof(ndev->common.stats));

	pci_set_drvdata(pcidev, ndev);

	/* setup timeout timer */
#if defined(timer_setup)
	timer_setup(&ndev->rd_timer, nfp_read_timeout, 0);
	mod_timer(&ndev->rd_timer, jiffies + (NFP_TIMEOUT_SEC * HZ));
#else
	init_timer(&ndev->rd_timer);
	ndev->rd_timer.function = nfp_read_timeout;
	ndev->rd_timer.data = (unsigned long)ndev;
	ndev->rd_timer.expires = jiffies + (NFP_TIMEOUT_SEC * HZ);
#endif

	nfp_dev_list[nfp_num_devices] = ndev;
	device_create(nfp_class, NULL, /* parent */
		      MKDEV(NFP_MAJOR, nfp_num_devices), NULL, /* drvdata */
		      "nshield_solo%d", nfp_num_devices);
	nfp_log(NFP_DBG2, "%s: nfp_num_devices= %d, ndev = %p.",
		__func__,
		nfp_num_devices, ndev);
	nfp_num_devices++;
	return 1;

fail_continue:
	nfp_dev_destroy(ndev, pcidev);

	return 0;
}

/* device probing ---------------------------------------------------------- */

/**
 * Adds a PCI device to the module. The PCI subsystem calls this function
 * when a PCI device is found.
 *
 * @param pcidev PCI device.
 * @param id PCI device ids.
 * @returns 0 if successful.
 */
static int nfp_pci_probe(struct pci_dev *pcidev,
			 struct pci_device_id const *id)
{
	int i;
	unsigned int bar[6];
	const struct nfpcmd_dev *cmddev = nfp_drvlist[id->driver_data];
	unsigned long iosize;
	unsigned int irq_line;
	int pos = 0u;
	int err = NFP_SUCCESS;

	if (!pcidev || !id) {
		nfp_log(NFP_DBG1, "%s: pcidev or id was NULL!", __func__);
		return -ENODEV;
	}

	nfp_log(NFP_DBG3, "%s: probing PCI device %s",
		__func__,
		pci_name(pcidev));

	/* enable the device */

	err = pci_enable_device(pcidev);
	if (err) {
		nfp_log(NFP_DBG1, "%s: pci_enable_device failed", __func__);
		err = -ENODEV;
		goto probe_err;
	}

	pci_set_master(pcidev);

	/* save PCI device info */

	irq_line = pcidev->irq;
	for (i = 0; i < NFP_BARSIZES_COUNT; ++i) {
		iosize = cmddev->bar_sizes[i] & NFP_BARSIZES_MASK;
		if (pci_resource_len(pcidev, i) < iosize) {
			nfp_log(NFP_DBG1,
				"%s: %s region request overflow: bar %d, requested %x, maximum %x",
				__func__,
				pci_name(pcidev), i, iosize,
				pci_resource_len(pcidev, i));
			err = -ENODEV;
			goto probe_err;
		}
		bar[i] = pci_resource_start(pcidev, i);
	}

	if (cmddev->flags & NFP_CMD_FLG_NEED_MSI) {
		pos = pci_find_capability(pcidev, PCI_CAP_ID_MSI);
		if (!pos) {
			nfp_log(NFP_DBG1, "%s: %s MSI not supported",
				__func__,
				pci_name(pcidev));
			err = -ENODEV;
			goto probe_err;
		}
		nfp_log(NFP_DBG1, "%s: %s MSI support at %d",
			__func__,
			pci_name(pcidev), pos);

		err = pci_enable_msi(pcidev);
		if (err) {
			nfp_log(NFP_DBG1,
				"%s: %s unable to enable MSI",
				__func__,
				pci_name(pcidev));
			goto probe_err;
		}

		/* IRQ vector changes if MSI is enabled. */
		irq_line = pcidev->irq;
		nfp_log(NFP_DBG3, "%s: %s MSI IRQ at %d",
			__func__,
			pci_name(pcidev), irq_line);
	}

	nfp_log(NFP_DBG2, "%s: devname %s, slotname %s, busname %s",
		__func__, "",
		pci_name(pcidev), pcidev->bus->name);

	err = nfp_setup(cmddev, pcidev->bus->number, PCI_SLOT(pcidev->devfn),
			bar, irq_line, pcidev);
	if (!err) {
		err = -ENODEV;
		goto probe_err;
	}

	return 0;

probe_err:
	pci_disable_msi(pcidev);
	pci_clear_master(pcidev);
	pci_disable_device(pcidev);
	return err;
}

/**
 * Removes a PCI device from the module. The PCI subsystem calls this function
 * when a PCI device is removed.
 *
 * @param pcidev PCI device.
 * @returns 0 if successful.
 */
static void nfp_pci_remove(struct pci_dev *pcidev)
{
	int index;
	struct nfp_dev *ndev;

	nfp_log(NFP_DBG1, "%s: removing PCI device %s",
		__func__,
		pci_name(pcidev));

	/* find existing device */

	ndev = pci_get_drvdata(pcidev);
	if (!ndev) {
		nfp_log(NFP_DBG1,
			"%s: no NSHIELD SOLO device associated with this PCI device",
			__func__);
		return;
	}

	/* destroy common device */

	if (ndev->cmddev)
		ndev->cmddev->destroy(ndev->common.cmdctx);

	nfp_dev_destroy(ndev, pcidev);

	pci_disable_msi(pcidev);
	pci_clear_master(pcidev);
	pci_disable_device(pcidev);

	index = 0;
	while (index < NFP_MAXDEV) {
		if (nfp_dev_list[index] == ndev) {
			nfp_dev_list[index] = NULL;
			device_destroy(nfp_class, MKDEV(NFP_MAJOR, index));
		}
		index++;
	}
}

/**
 * PCI device ID table.
 */
static struct pci_device_id nfp_pci_tbl[] = {
	{
		PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_21555,
		PCI_VENDOR_ID_NCIPHER, PCI_SUBSYSTEM_ID_NFAST_REV1, 0,
		0, /* Ignore class */
		0 /* Index into nfp_drvlist */
	},
	{
		PCI_VENDOR_ID_FREESCALE, PCI_DEVICE_ID_FREESCALE_C293,
		PCI_VENDOR_ID_NCIPHER, PCI_SUBSYSTEM_ID_NFAST_REV1, 0,
		0, /* Ignore class */
		1 /* Index into nfp_drvlist */
	},
	{
		PCI_VENDOR_ID_FREESCALE, PCI_DEVICE_ID_FREESCALE_P3041,
		PCI_VENDOR_ID_NCIPHER, PCI_SUBSYSTEM_ID_NFAST_REV1, 0,
		0, /* Ignore class */
		2 /* Index into nfp_drvlist */
	},
	{
		PCI_VENDOR_ID_FREESCALE, PCI_DEVICE_ID_FREESCALE_T1022,
		PCI_VENDOR_ID_NCIPHER, PCI_SUBSYSTEM_ID_NFAST_REV1, 0,
		0, /* Ignore class */
		3 /* Index into nfp_drvlist */
	},
	{
		0,
	} /* terminate list */
};
MODULE_DEVICE_TABLE(pci, nfp_pci_tbl);

/**
 * PCI driver operations.
 */
static struct pci_driver nfp_pci_driver = { .name = "nshield_solo",
					    .probe = nfp_pci_probe,
					    .remove =
						    __devexit_p(nfp_pci_remove),
					    .id_table = nfp_pci_tbl };

/*--------------------*/
/*  init              */
/*--------------------*/

static int nfp_init(void)
{
	int index;

	nfp_log(NFP_DBG1, "%s: entered", __func__);

	if (register_chrdev(NFP_MAJOR, NFP_DRVNAME, &nfp_fops)) {
		nfp_log(NFP_DBG1,
			"unable to get major for nshield_solo device.");
		return -EIO;
	}

	for (index = 0; index < NFP_MAXDEV; index++)
		nfp_dev_list[index] = NULL;

	nfp_class = class_create(THIS_MODULE, "nshield_solo");
	if (IS_ERR(nfp_class)) {
		nfp_log(NFP_DBG1,
			"%s: failed to create a class for this device, err = %ld",
			__func__,
			PTR_ERR(nfp_class));
		return -EIO;
	}

	index = 0;
	return pci_register_driver(&nfp_pci_driver);
}

/** @} */

/**
 * Initializes this NSHIELD SOLO kernel module.
 */
static int __init nfp_module_init(void)
{
	int err;

	nfp_log(NFP_DBG1, "%s: inserting nshield_solo module", __func__);

	err = nfp_init();

	return err;
}

/**
 * Exits this NSHIELD SOLO kernel module.
 */
static void __exit nfp_module_exit(void)
{
	nfp_log(NFP_DBG3, "%s: removing nshield_solo module", __func__);

	/* unregister pci driver */

	pci_unregister_driver(&nfp_pci_driver);
	/* ... which triggers device removals */

	class_destroy(nfp_class);

	unregister_chrdev(NFP_MAJOR, NFP_DRVNAME);
	nfp_log(NFP_DBG1, "%s: removed nshield_solo module", __func__);
}

module_init(nfp_module_init);
module_exit(nfp_module_exit);

/* end of file */
