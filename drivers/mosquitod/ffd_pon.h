/*
 *  ffd_pon.h
 *  
 */
#ifndef __FFD_PON_H__
#define __FFD_PON_H__

#ifdef __cplusplus
extern "C" {
#endif
/*
 *  INCLUDE FILES FOR MODULE
 *  
 */
#include <linux/list.h>
#include "ffd_common.h"

int ffd_pon_init(void);
void ffd_pon_exit(void);
	
#ifdef __cplusplus
}
#endif

#endif // __FFD_PON_H__
