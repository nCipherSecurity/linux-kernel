/*

nfdev-linux.h: nFast linux specific device ioctl interface.

(C) Copyright COMPANY Corporation Ltd 1998 All rights reserved

history

14/07/1998 jsh  Original

*/

#ifndef NFDEV_LINUX_H
#define NFDEV_LINUX_H

#include "nfdev-common.h"

#define NFDEV_IOCTL_TYPE 0x10

#define NFDEV_IOCTL_CHUPDATE		_IO( NFDEV_IOCTL_TYPE, \
					     NFDEV_IOCTL_NUM_CHUPDATE )

#define NFDEV_IOCTL_ENQUIRY		_IOR( NFDEV_IOCTL_TYPE, \
					      NFDEV_IOCTL_NUM_ENQUIRY, \
					       nfdev_enquiry_str )

#define NFDEV_IOCTL_ENSUREREADING	_IOW( NFDEV_IOCTL_TYPE, \
					      NFDEV_IOCTL_NUM_ENSUREREADING, \
					      int )

#define NFDEV_IOCTL_ENSUREREADING_BUG3349	_IO( NFDEV_IOCTL_TYPE, \
					     NFDEV_IOCTL_NUM_ENSUREREADING )


#define NFDEV_IOCTL_DEBUG		_IOW( NFDEV_IOCTL_TYPE, \
					      NFDEV_IOCTL_NUM_DEBUG, \
					      int )

#define NFDEV_IOCTL_PCI_IFVERS		_IOW( NFDEV_IOCTL_TYPE, \
					      NFDEV_IOCTL_NUM_PCI_IFVERS, \
					      int )

#define NFDEV_IOCTL_STATS		_IOR( NFDEV_IOCTL_TYPE, \
					      NFDEV_IOCTL_NUM_STATS, \
					       nfdev_stats_str )

#define NFDEV_IOCTL_CONTROL		_IOW( NFDEV_IOCTL_TYPE, \
					      NFDEV_IOCTL_NUM_CONTROL, \
					      const nfdev_control_str )
 
#define NFDEV_IOCTL_STATUS		_IOR( NFDEV_IOCTL_TYPE, \
					      NFDEV_IOCTL_NUM_STATUS, \
					      nfdev_status_str )

#endif /* NFDEV_LINUX_H */
