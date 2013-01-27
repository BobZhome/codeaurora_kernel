/*
 *  ffd_pon.c
 *  
 */
 
/*
 *    INCLUDE FILES FOR MODULE
 */
#include "ffd_pon.h"
#include "ffd_gpio.h"

/*
 *  DEFINE
 */

/*
 *    INTERNAL DEFINITION
 */

static int isopen = 0; // 0 : No open 1 : Open

/*
 *    FUNCTION DEFINITION
 */

/*
* Description : MFC calls this function using close method(void open()) of FileOutputStream class
*               When this fuction is excuted, set PON to Low.
* Input : None
* Output : Success : 0 Fail : Other
*/
static int ffd_pon_open (struct inode *inode, struct file *fp)
{
  int rc = 0;

  if(1 == isopen)
  {
    #ifdef FFD_DEBUG_LOW 
    FFD_DEBUG_MSG("[FFD] ffd_pon_open - already open \n");
    #endif
    return -1;
  }
  else
  {
    #ifdef FFD_DEBUG_LOW 
    FFD_DEBUG_MSG("[FFD] ffd_pon_open - start \n");
    #endif
    isopen = 1;
  }

  rc = ffd_gpio_open(GPIO_FFD_PON, GPIO_DIRECTION_OUT, GPIO_LOW_VALUE);


  #ifdef FFD_DEBUG_LOW 
  FFD_DEBUG_MSG("[FFD] ffd_pon_open - end \n");
  #endif

  return rc;
}

/*
* Description : MFC calls this function using write method(void write(int oneByte)) of FileOutputStream class
* Input : PON low : 0 PON high : 1
* Output : Success : 0 Fail : Other
*/
static ssize_t ffd_pon_write(struct file *fp, const char *buf, size_t count, loff_t *pos)
{
  int rc = 0;
  int SetValue = 0;


  #ifdef FFD_DEBUG_LOW 
  FFD_DEBUG_MSG("[FFD] ffd_pon_write - start \n");
  #endif

/* Check error */
  if(NULL == fp)
  {
    FFD_DEBUG_MSG("[FFD] ERROR file \n");
    return -1;    
  }

  if(NULL == buf)
  {
    FFD_DEBUG_MSG("[FFD] ERROR buf \n");
    return -1;    
  }
  
  if(1 != count)
  {
    FFD_DEBUG_MSG("[FFD]ERROR count \n");
    return -1;    
  }

  if(NULL == pos)
  {
    FFD_DEBUG_MSG("[FFD] ERROR file \n");
    return -1;    
  }

  rc = copy_from_user(&SetValue, (void*)buf, count);
  if(rc)
  {
    FFD_DEBUG_MSG("[FFD] ERROR - copy_from_user \n");
    return rc;
  }

  if((GPIO_LOW_VALUE != SetValue)&&(GPIO_HIGH_VALUE != SetValue))
  {
    FFD_DEBUG_MSG("[FFD] ERROR - SetValue is out of range \n");
    return -1;
  }
  else if(GPIO_LOW_VALUE != SetValue)
  {
    FFD_DEBUG_MSG("[FFD] ========> ON \n");
  }
  else if(GPIO_HIGH_VALUE != SetValue)
  {
    FFD_DEBUG_MSG("[FFD] <======== OFF \n");
  }

  ffd_gpio_write(GPIO_FFD_PON, SetValue);

  /* you gotta fuck yourself, modyfing this line */
  mdelay(20);

  return 1;
}

/*
* Description : MFC calls this function using close method(void close()) of FileOutputStream class
*               When this fuction is excuted, set PON to Low.
* Input : None
* Output : Success : 0 Fail : Other
*/
static int ffd_pon_release (struct inode *inode, struct file *fp)
{
	
  if(0 == isopen)
  {
    #ifdef FFD_DEBUG_LOW 
    FFD_DEBUG_MSG("[FFD] ffd_pon_release - not open \n");
    #endif

    return -1;
  }
  else
  {
    #ifdef FFD_DEBUG_LOW 
    FFD_DEBUG_MSG("[FFD] ffd_pon_release - start \n");
    #endif

    isopen = 0;
  }

  FFD_DEBUG_MSG("[FFD] ffd_pon_release  <======== OFF \n");
  ffd_gpio_write(GPIO_FFD_PON, GPIO_LOW_VALUE);

  #ifdef FFD_DEBUG_LOW 
  FFD_DEBUG_MSG("[FFD] ffd_pon_release - end \n");
  #endif

  return 0;
}

/*
 *    STRUCT DEFINITION
 */

static struct file_operations ffd_pon_fops = 
{
  .owner    = THIS_MODULE,
  .open      = ffd_pon_open,
  .write    = ffd_pon_write,
  .release  = ffd_pon_release,
};

static struct miscdevice ffd_pon_device = 
{
  .minor = 251,
  .name = FFD_PON_NAME,
  .fops = &ffd_pon_fops,
};

int ffd_pon_init(void)
{
  int rc = 0;

  #ifdef FFD_DEBUG_LOW 
  FFD_DEBUG_MSG("[FFD] ffd_pon_init - start \n");
  #endif

  /* register the device file */
  rc = misc_register(&ffd_pon_device);
  if (rc)
  {
    FFD_DEBUG_MSG("[FFD] FAIL!! can not register ffd_pon \n");
    return rc;
  }
  
  #ifdef FFD_DEBUG_LOW 
  FFD_DEBUG_MSG("[FFD] ffd_pon_init - end \n");
  #endif

  return 0;
}

void ffd_pon_exit(void)
{
  #ifdef FFD_DEBUG_LOW 
  FFD_DEBUG_MSG("[FFD] ffd_pon_exit - start \n");
  #endif

  /* deregister the device file */
  misc_deregister(&ffd_pon_device);
  
  #ifdef FFD_DEBUG_LOW 
  FFD_DEBUG_MSG("[FFD] ffd_pon_exit - end \n");
  #endif
}
