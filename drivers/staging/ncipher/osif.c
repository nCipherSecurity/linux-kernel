// SPDX-License-Identifier: GPL-2.0+
/*
 *
 * osif.c: nCipher PCI HSM OS interface
 *
 */

#include <linux/spinlock.h>
#include <linux/slab.h>

#include "osif.h"

void nfp_sleep(int ms)
{
	DEFINE_WAIT(wait);
	wait_queue_head_t q;

	init_waitqueue_head(&q);

	prepare_to_wait(&q, &wait, TASK_UNINTERRUPTIBLE);
	schedule_timeout((ms * HZ) / 1000);
	finish_wait(&q, &wait);
}

int nfp_config_inl(struct nfp_dev *ndev, int offset, u32 *res)
{
	if (!ndev || !ndev->pcidev)
		return NFP_ENODEV;
	pci_read_config_dword(ndev->pcidev, offset, res);
	return 0;
}

/* user space memory access ---------------------------------- */

int nfp_copy_from_user(char *kbuf, const char *ubuf, int len)
{
	int oserr;

	oserr = copy_from_user(kbuf, ubuf, len) ? -EFAULT : 0;
	return nfp_error(oserr);
}

int nfp_copy_to_user(char *ubuf, const char *kbuf, int len)
{
	int oserr;

	oserr = copy_to_user(ubuf, kbuf, len) ? -EFAULT : 0;
	return nfp_error(oserr);
}

int nfp_copy_from_user_to_dev(struct nfp_dev *ndev, int bar, int offset,
			      const char *ubuf, int len)
{
	int oserr;

	oserr = copy_to_user(ndev->bar[bar] + offset, ubuf, len) ? -EFAULT : 0;
	return nfp_error(oserr);
}

int nfp_copy_to_user_from_dev(struct nfp_dev *ndev, int bar, int offset,
			      char *ubuf, int len)
{
	int oserr;

	oserr = copy_to_user(ubuf, ndev->bar[bar] + offset, len) ? -EFAULT : 0;
	return nfp_error(oserr);
}

int nfp_copy_from_dev(struct nfp_dev *ndev, int bar,
		      int offset, char *kbuf, int len)
{
	memcpy(kbuf, ndev->bar[bar] + offset, len);
	return NFP_SUCCESS;
}

int nfp_copy_to_dev(struct nfp_dev *ndev, int bar,
		    int offset, const char *kbuf, int len)
{
	memcpy(ndev->bar[bar] + offset, kbuf, len);
	return NFP_SUCCESS;
}

/* pci fixed length accessors. The below functions are used predominantly
 * to access CSR registers in pci memory space.
 */
u32 nfp_inl(struct nfp_dev *ndev, int bar, int offset)
{
	nfp_log(NFP_DBG3, "%s: addr %p", __func__, ndev->bar[bar] + offset);
	return ioread32(ndev->bar[bar] + offset);
}

u16 nfp_inw(struct nfp_dev *ndev, int bar, int offset)
{
	nfp_log(NFP_DBG3, "%s: addr %p", __func__, ndev->bar[bar] + offset);
	return ioread16(ndev->bar[bar] + offset);
}

void nfp_outl(struct nfp_dev *ndev, int bar, int offset, u32 data)
{
	nfp_log(NFP_DBG3, "%s: addr %p, data %x", __func__,
		ndev->bar[bar] + offset, data);
	iowrite32(data, ndev->bar[bar] + offset);
}

void nfp_outw(struct nfp_dev *ndev, int bar, int offset, u16 data)
{
	nfp_log(NFP_DBG3, "%s: addr %p, data %x", __func__,
		ndev->bar[bar] + offset, data);
	iowrite16(data, ndev->bar[bar] + offset);
}

/* logging ---------------------------------------------------- */

void nfp_log(int level, const char *fmt, ...)
{
	va_list ap;

	switch (level) {
	case NFP_DBG4:
		if (nfp_debug < 4)
			break;
	/*FALLTHROUGH*/
	case NFP_DBG3:
		if (nfp_debug < 3)
			break;
	/*FALLTHROUGH*/
	case NFP_DBG2:
		if (nfp_debug < 2)
			break;
	/*FALLTHROUGH*/
	case NFP_DBG1:
		if (nfp_debug < 1)
			break;
	/*FALLTHROUGH*/
	default:
		va_start(ap, fmt);
		vprintk(fmt, ap);
		va_end(ap);

		break;
	}
}

struct errstr {
	int oserr;
	int nferr;
};

static struct errstr errtab[] = { { -EFAULT, NFP_EFAULT },
				  { -ENOMEM, NFP_ENOMEM },
				  { -EINVAL, NFP_EINVAL },
				  { -EIO, NFP_EIO },
				  { -ENXIO, NFP_ENXIO },
				  { -ENODEV, NFP_ENODEV },
				  { 0, 0 } };

int nfp_error(int oserr)
{
	struct errstr *perr;

	if (!oserr)
		return 0;
	perr = errtab;
	while (perr->nferr) {
		if (perr->oserr == oserr)
			return perr->nferr;
		perr++;
	}
	return NFP_EUNKNOWN;
}

int nfp_oserr(int nferr)
{
	struct errstr *perr;

	if (nferr == NFP_SUCCESS)
		return 0;
	perr = errtab;
	while (perr->nferr) {
		if (perr->nferr == nferr)
			return perr->oserr;
		perr++;
	}
	return -EIO;
}
