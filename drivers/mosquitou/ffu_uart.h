/*
 *  ffu_uart.h
 *  
 */

#ifndef __FFU_UART_H__
#define __FFU_UART_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  INCLUDE FILES FOR MODULE
 */
#include "ffu_common.h"

/*
 *  DEFINE
 */
int ffu_uart_open(void);
int ffu_uart_close(void);
int ffu_uart_write(char *buf, size_t count);
int ffu_uart_read(char *buf, size_t count);
int ffu_uart_ioctrl(int *count);

#ifdef __cplusplus
}
#endif

#endif // __FFU_UART_H__
