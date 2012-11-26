/*
 *  ffd_common.h
 *  
 */

#ifndef __FFD_COMMON_H__
#define __FFD_COMMON_H__

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


/*
 *  DEFINE
 */

/* debug message */
/* #define FFD_DEBUG_LOW */
#define FFD_DEBUG_MSG printk

#define FFD_PON_NAME    "mosquito_p"
#define FFD_NAME    "mosquito_d"

extern int ffu_uart_open(void);
extern int ffu_uart_close(void);
extern int ffu_uart_write(char *buf, size_t count);
extern int ffu_uart_read(char *buf, size_t count);
extern int ffu_uart_ioctrl(int *count);

#ifdef __cplusplus
}
#endif

#endif // __FFD_COMMON_H__
