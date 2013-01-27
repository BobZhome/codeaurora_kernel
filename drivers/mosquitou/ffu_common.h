/*
 *  ffu_common.h
 *  
 */

#ifndef __FFU_COMMON_H__
#define __FFU_COMMON_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  INCLUDE FILES FOR MODULE
 */
#include <linux/module.h>/*THIS_MODULE*/
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h>/* printk() */
#include <linux/types.h>/* size_t */
#include <linux/miscdevice.h>/*misc_register, misc_deregister*/
#include <linux/vmalloc.h>
#include <linux/fs.h>/*file_operations*/
#include <linux/delay.h>/*mdelay*/

#include <asm/uaccess.h>/*copy_from_user*/
#include <asm/io.h>/*static*/
#include <mach/gpio.h>

/* debug message */
/* #define FFU_DEBUG_LOW */
#define FFU_DEBUG_MSG printk

#ifdef __cplusplus
}
#endif

#endif // __FFU_COMMON_H__
