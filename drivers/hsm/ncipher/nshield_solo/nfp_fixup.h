/* SPDX-License-Identifier: GPL-2.0+ */
/*
 *
 * nfp_fixup.h: nCipher PCI HSM linux version fixup macros
 *
 */

#ifndef FIXUP_H
#define FIXUP_H

#define VERSION(ver, rel, seq) (((ver) << 16) | ((rel) << 8) | (seq))
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

#define NFP_MAJOR 176 /* "nCipher nFast PCI crypto accelerator" */

#define COPY_FROM_USER(DST, SRC, LEN, error)                                   \
	(error = copy_from_user(DST, SRC, LEN) ? -EFAULT : 0)
#define COPY_TO_USER(DST, SRC, LEN, error)                                     \
	(error = copy_to_user(DST, SRC, LEN) ? -EFAULT : 0)
#define INODE_FROM_FILE(file) ((file)->f_path.dentry->d_inode)

#define NFP_TIMEOUT ((NFP_TIMEOUT_SEC) * HZ)

#endif
