/*

nfp_fixup.h: COMPANY PCI HSM linux version fixup macros

  * COPYRIGHT

history

09/10/2001 jsh  Original

*/

#ifndef FIXUP_H
#define FIXUP_H

#define VERSION(ver,rel,seq) (((ver)<<16) | ((rel)<<8) | (seq))
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

#define pci_module_init pci_register_driver

#include <linux/init.h>
#if LINUX_VERSION_CODE < VERSION(2,6,39)
#define spinlock_init(plock)  *(plock) = SPIN_LOCK_UNLOCKED
#else
#define spinlock_init(plock) *(plock) = __SPIN_LOCK_UNLOCKED(*(plock))
#endif

#include <linux/moduleparam.h>
#define NFP_MODULE_PARAMETERS \
module_param(nfp_debug,int,0644); \
module_param(nfp_ifvers,int,0444);
#define NFP_MODULE_PREAMBLE \
  MODULE_AUTHOR("COMPANY"); \
  MODULE_DESCRIPTION("COMPANY PCI HSM driver"); \
  NFP_MODULE_PARAMETERS \
  MODULE_PARM_DESC(nfp_debug,"debug level (1-4)"); \
  MODULE_PARM_DESC(nfp_ifvers,"maximum interface version (1-2), or any (0)");

typedef wait_queue_head_t nfp_wait_queue_head_t;
#if LINUX_VERSION_CODE < VERSION(4,13,0)
typedef wait_queue_t nfp_wait_queue_t;
#else
typedef wait_queue_entry_t nfp_wait_queue_t;
#endif

#define nfp_init_waitqueue_head(x)    init_waitqueue_head(x)
#define nfp_init_waitqueue_entry(x,y) init_waitqueue_entry(x,y)
#define nfp_wake_up_all(x)          wake_up_all(x)

#define NFP_MAJOR       176

/* NB: most of the 2.1.x dependent code is untested */

#define COPY_FROM_USER(DST,SRC,LEN,error) error = copy_from_user(DST,SRC,LEN) ? -EFAULT : 0
#define COPY_TO_USER(DST,SRC,LEN,error)   error = copy_to_user(DST,SRC,LEN) ? -EFAULT : 0
#if LINUX_VERSION_CODE >= VERSION(3,19,0)
  #define INODE_FROM_FILE( file )       ((file)->f_path.dentry->d_inode)
#else
  #define INODE_FROM_FILE( file )       ((file)->f_dentry->d_inode)
#endif

#include <linux/poll.h>
typedef ssize_t read_write_t;
typedef int release_t;
typedef size_t count_t;

#define RELEASE_RETURN( x ) return x
#define GET_FREE_PAGES( x, y, z ) __get_free_pages( x, y )

#include <asm/uaccess.h>

#define NFP_TIMEOUT ((NFP_TIMEOUT_SEC) * HZ)
#define IOREMAP(ADDR, LEN)            ioremap(ADDR, LEN)
#define IOREMAP_NOCACHE(ADDR, LEN)    ioremap_nocache(ADDR, LEN)
  #define IOUNMAP(ADDR)                 iounmap(ADDR)
  #define nfp_schedule_timeout(time, timeout) \
    timeout = schedule_timeout(time);

#ifdef MODULE_LICENSE
#define NFP_MODULE_LICENSE \
  MODULE_LICENSE("GPL");
#else
#define NFP_MODULE_LICENSE
#endif

#endif


