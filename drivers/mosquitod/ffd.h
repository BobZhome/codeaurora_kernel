/*
 *  ffd.h
 *  
 */
#ifndef __FFD_H__
#define __FFD_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  INCLUDE FILES FOR MODULE
 *  
 */

#include <linux/list.h>

#include "ffd_common.h"

/*
 *  DEFINE
 *  
 */

#define IOCTL_FFD_MAGIC 'T'
#define IOCTL_FFD_CMD_AVAILABLE 0x1B

enum
{
  IOCTL_MAGIC_0 = 0,
  IOCTL_MAGIC_1,
  IOCTL_MAGIC_MAX,
};

#define IOCTL_FFD_AVAILABLE  _IO(IOCTL_FFD_MAGIC, IOCTL_FFD_CMD_AVAILABLE)

int ffd_init(void);
void ffd_exit(void);

#ifdef __cplusplus
}
#endif

#endif // __FFD_H__
