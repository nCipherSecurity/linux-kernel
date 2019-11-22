// SPDX-License-Identifier: GPL-2.0+
/*
 *
 * osif.c: nCipher PCI HSM OS interface
 *
 */

#include "solo.h"

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
	return oserr;
}

int nfp_copy_to_user_from_dev(struct nfp_dev *ndev, int bar, int offset,
			      char *ubuf, int len)
{
	int oserr;

	oserr = copy_to_user(ubuf, ndev->bar[bar] + offset, len) ? -EFAULT : 0;
	return oserr;
}

int nfp_copy_from_dev(struct nfp_dev *ndev, int bar,
		      int offset, char *kbuf, int len)
{
	memcpy(kbuf, ndev->bar[bar] + offset, len);
	return 0;
}

int nfp_copy_to_dev(struct nfp_dev *ndev, int bar,
		    int offset, const char *kbuf, int len)
{
	memcpy(ndev->bar[bar] + offset, kbuf, len);
	return 0;
}
