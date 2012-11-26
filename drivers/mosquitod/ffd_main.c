/*
 *  ffd_main.c
 *  
 */
 
/*
 *    INCLUDE FILES FOR MODULE
 */
#include "ffd_main.h"
/*
 *    FUNCTION DEFINITION
 */

static int ffd_main_init(void)
{
  int rc = 0;

  /* register the device file */
  rc = ffd_pon_init();
  if (rc)
  {
    FFD_DEBUG_MSG("[FFD_MAIN] FAIL!! can not register ffd_pon \n");
    return rc;
  }

  /* register the device file */
  rc = ffd_init();
  if (rc)
  {
    FFD_DEBUG_MSG("[FFD_MAIN] FAIL!! can not register ffd\n");
    return rc;
  }
	
		
  #ifdef FFD_DEBUG_LOW 
  FFD_DEBUG_MSG("[FFD_MAIN] ffd_main_init - end \n");
  #endif

  return 0;
}

static void ffd_main_exit(void)
{
  #ifdef FFD_DEBUG_LOW 
  FFD_DEBUG_MSG("[FFD_MAIN] ffd_main_exit - start \n");
  #endif

  /* deregister the device file */
  ffd_pon_exit();
  ffd_exit();
	
  #ifdef FFD_DEBUG_LOW 
  FFD_DEBUG_MSG("[FFD_MAIN] ffd_main_exit - end \n");
  #endif
}

module_init(ffd_main_init);
module_exit(ffd_main_exit);

MODULE_LICENSE("Dual BSD/GPL");