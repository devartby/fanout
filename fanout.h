#ifndef _fanout_h_
#define _fanout_h_

#include <asm/ioctl.h>

#define FANOUT_IOCTL_BASE 'f'

#define FANOUT_GET_RD _IOR(FANOUT_IOCTL_BASE, 0, int)
#define FANOUT_GET_WR _IOR(FANOUT_IOCTL_BASE, 1, int)
#define FANOUT_SET_SEEK_SET _IOW(FANOUT_IOCTL_BASE, 2, int)
#define FANOUT_SET_SEEK_ALIGNMENT _IOW(FANOUT_IOCTL_BASE, 3, int)


#endif
