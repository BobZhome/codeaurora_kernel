/* arch/arm/mach-msm/include/mach/lge_diag_test.h
 * Copyright (C) 2010 LGE Corporation.
 * 
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

<<<<<<< HEAD
#ifndef __ARCH_MSM_LGE_DIAG_TEST_H
#define __ARCH_MSM_LGE_DIAG_TEST_H

extern uint8_t if_condition_is_on_key_buffering;
extern uint8_t lgf_factor_key_test_rsp(char);
extern unsigned int ats_mtc_log_mask;

extern int eta_execute(char *);
extern int base64_encode(char *, int, char *);

#endif
=======
#include <stddef.h>

#ifndef __ARCH_MSM_LGE_DIAG_TEST_H
#define __ARCH_MSM_LGE_DIAG_TEST_H

#define eta_execute(str) printk("Warning.  Using unsafe function eta_execute. %s:%d.", __FILE__, __LINE__); eta_execute_n(str, strlen(str))

extern uint8_t if_condition_is_on_key_buffering;
extern uint8_t lgf_factor_key_test_rsp(char);

extern int eta_execute_n(char *, size_t);
extern int base64_encode(char *, int, char *);

#endif

>>>>>>> vendor-vs660-froyo
