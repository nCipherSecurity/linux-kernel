// SPDX-License-Identifier: GPL-2.0+
/*
 *
 * osif.c: nCipher PCI HSM OS interface
 *
 */

#include "solo.h"

/* error conversion ------------------------------------------- */
int     nfp_error(int oerr);

void nfp_sleep(int ms)
{
	DEFINE_WAIT(wait);
	wait_queue_head_t q;

	init_waitqueue_head(&q);

	prepare_to_wait(&q, &wait, TASK_UNINTERRUPTIBLE);
	schedule_timeout((ms * HZ) / 1000);
	finish_wait(&q, &wait);
}

/* user space memory access ---------------------------------- */

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

struct errstr errtab[] = { { -EFAULT, NFP_EFAULT },
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
