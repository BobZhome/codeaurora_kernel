/*
 *  ffd_gpio.c
 *  
 */
 
/*
 *    INCLUDE FILES FOR MODULE
 */

#include "ffd_gpio.h"

#include <linux/gpio.h>

/*
* Description : 
* Input : 
* Output : 
*/
int ffd_gpio_open(int gpionum, int direction, int value)
{
  int rc = 0;
  
#ifdef FELICA_GPIO_L_MODEL
  unsigned gpio_cfg = 0;

  if(direction == GPIO_DIRECTION_IN)
  {
    gpio_cfg =	FELICA_GPIO_CFG(gpionum, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA);

    rc = gpio_tlmm_config(gpio_cfg, GPIO_CONFIG_ENABLE);

    if(rc)
    {
      FFD_DEBUG_MSG("[FELICA] ERROR - gpio_tlmm_config \n");
      return rc;
    }

    rc = gpio_direction_input(gpionum);

    if(rc)
    {
      FFD_DEBUG_MSG("[FELICA] ERROR -	gpio_direction_input \n");
      return rc;
    }
  }
  else
  {
    gpio_cfg =	FELICA_GPIO_CFG(gpionum, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);

    rc = gpio_tlmm_config(gpio_cfg, GPIO_CONFIG_ENABLE);

    if(rc)
    {
      FFD_DEBUG_MSG("[FELICA] ERROR - gpio_tlmm_config \n");
      return rc;
    }

    rc = gpio_direction_output(gpionum, GPIO_HIGH_VALUE);
    
    if(rc)
    {
      FFD_DEBUG_MSG("[FELICA] ERROR -	gpio_direction_output \n");
      return rc;
    }
  }
#else
  /* Configure GPIO */
  rc = gpio_tlmm_config(gpionum, GPIO_CONFIG_ENABLE);
  if(rc)
  {
    FFD_DEBUG_MSG("[FFD] ERROR - gpio_tlmm_config \n");
    return rc;
  }

  /* Set output direction */
  if(GPIO_DIRECTION_IN == direction)
  {
    rc = gpio_direction_input(gpionum);
    if(rc)
    {
      FFD_DEBUG_MSG("[FFD] ERROR -  gpio_direction_input \n");
      return rc;
    }  
  }
  else
  {
    rc = gpio_direction_output(gpionum, value);
    if(rc)
    {
      FFD_DEBUG_MSG("[FFD] ERROR -  gpio_direction_output \n");
      return rc;
    }
  }
#endif

  return rc;
}

/*
* Description : 
* Input : 
* Output : 
*/
void ffd_gpio_write(int gpionum, int value)
{
  gpio_set_value(gpionum, value);
}

/*
* Description : 
* Input : 
* Output : 
*/
int ffd_gpio_read(int gpionum)
{
  return gpio_get_value(gpionum);
}

