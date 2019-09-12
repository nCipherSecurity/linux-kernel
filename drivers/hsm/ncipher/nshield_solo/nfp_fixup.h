/*
 *
 * nfp_fixup.h: nCipher PCI HSM linux version fixup macros
 *
 * (c) nCipher Security Limited 2019
 *
 * history
 *
 * 09/10/2001 jsh  Original
 *
 */

#ifndef FIXUP_H
#define FIXUP_H

#define VERSION(ver, rel, seq) (((ver) << 16) | ((rel) << 8) | (seq))
#include <linux/version.h>

#include <linux/types.h>
#include <linux/time.h>
#include <asm/io.h>
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

// typedef wait_queue_head_t nfp_wait_queue_head_t;
#if LINUX_VERSION_CODE < VERSION(4, 13, 0)
typedef wait_queue_t nfp_wait_queue_t;
#else
typedef wait_queue_entry_t nfp_wait_queue_t;
#endif

#define NFP_MAJOR 176

/* NB: most of the 2.1.x dependent code is untested */

#define COPY_FROM_USER(DST, SRC, LEN, error)                                   \
	error = copy_from_user(DST, SRC, LEN) ? -EFAULT : 0
#define COPY_TO_USER(DST, SRC, LEN, error)                                     \
	error = copy_to_user(DST, SRC, LEN) ? -EFAULT : 0
#if LINUX_VERSION_CODE >= VERSION(3, 19, 0)
#define INODE_FROM_FILE(file) ((file)->f_path.dentry->d_inode)
#else
#define INODE_FROM_FILE(file) ((file)->f_dentry->d_inode)
#endif

#define NFP_TIMEOUT ((NFP_TIMEOUT_SEC) * HZ)

#endif
