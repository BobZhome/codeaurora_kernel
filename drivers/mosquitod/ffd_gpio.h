/*
 *  ffd_agpio.h
 *  
 */

#ifndef __FFD_GPIO_H__
#define __FFD_GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  INCLUDE FILES FOR MODULE
 */

#include <linux/list.h>

#include "ffd_common.h"
/*
 *  DEFINE
 */
/* MODEL NAME */
#define FELICA_GPIO_L_MODEL

/* common */ 
enum{
  GPIO_DIRECTION_IN = 0,
  GPIO_DIRECTION_OUT,
};

enum{
  GPIO_LOW_VALUE = 0,
  GPIO_HIGH_VALUE,
};

#ifdef FELICA_GPIO_L_MODEL
enum{
  GPIO_CONFIG_ENABLE = 0,
  GPIO_CONFIG_DISABLE,
};
#else
enum{
  GPIO_CONFIG_DISABLE = 0,
  GPIO_CONFIG_ENABLE,
};
#endif

/* felica_pon */
#define GPIO_FFD_PON   89

#ifdef FELICA_GPIO_L_MODEL
#define FELICA_GPIO_CFG(gpio, func, dir, pull, drvstr) \
    ((((gpio) & 0x3FF) << 4)        |   \
    ((func) & 0xf)                  |   \
    (((dir) & 0x1) << 14)           |   \
    (((pull) & 0x3) << 15)          |   \
    (((drvstr) & 0xF) << 17))
#endif
/*
 *  FUNCTION PROTOTYPE
 */
int ffd_gpio_open(int gpionum, int direction, int value);
void ffd_gpio_write(int gpionum, int value);
int ffd_gpio_read(int gpionum);

#ifdef __cplusplus
}
#endif

#endif // __FFD_GPIO_H__
