/*
 *  ffd.c
 *  
 */
 
/*
 *    INCLUDE FILES FOR MODULE
 */
#include <linux/mutex.h>

#include "ffd.h"
/*
 *  DEFINE
 */
#define RXTX_LOG_ENABLE

/* Definition for transmit and receive buffer */
#define RECEIVE_BUFFER_MAX_SIZE 4096 /* FN requirement for felica device driver 1.7 */
#define TRANSMIT_BUFFER_MAX_SIZE 4096 

char receive_buf[RECEIVE_BUFFER_MAX_SIZE + 4];
char transmit_buf[TRANSMIT_BUFFER_MAX_SIZE + 4];

/*
 *   INTERNAL VARIABLE
 */
static int isopen = 0; // 0 : No open 1 : Open

static DEFINE_MUTEX(ffd_mutex);

/*
 *    FUNCTION PROTOTYPE
 */

/*
 *   FUNCTION DEFINITION
 */

/*
* Description :
* Input : 
* Output :
*/
static int ffd_open (struct inode *inode, struct file *fp)
{
  int rc = 0;


  #ifdef FFD_DEBUG_LOW 
  FFD_DEBUG_MSG("[FFD] ffd_open() is called \n");
  #endif

  /* Check input parameters */
  if(NULL == fp)
  {
    FFD_DEBUG_MSG("[FFD] ERROR - fp \n");
    return -1;  
  }

  /* FileInputStream and FileOutPutStream open felica
     Only one case has to be excuted */
  if(1 == isopen)
  {
    #ifdef FFD_DEBUG_LOW
    FFD_DEBUG_MSG("[FFD] /dev/ffd is already openned. \n");
    #endif

    return 0;
  }
  else
  {
    #ifdef FFD_DEBUG_LOW
    FFD_DEBUG_MSG("[FFD] ffd_open - start \n");
    #endif

    isopen = 1;
  }
  
  rc = ffu_uart_open();

  //mdelay(100);

  if(rc)
  {
    FFD_DEBUG_MSG("[FFD] ERROR - open_hs_uart \n");
    return rc;
  }

  #ifdef FFD_DEBUG_LOW
  FFD_DEBUG_MSG("[FFD] ffd_open - end \n");
  #endif

  return 0;
}

/*
* Description :
* Input : 
* Output :
*/

static ssize_t ffd_read(struct file *fp, char *buf, size_t count, loff_t *pos)
{
  int rc = 0;
  int readcount = 0;

 
  #ifdef FFD_DEBUG_LOW
  FFD_DEBUG_MSG("[FFD] ffd_read - start \n");
  #endif

  /* Check input parameters */
  if(NULL == fp)
  {
    FFD_DEBUG_MSG("[FFD] ERROR - fp \n");
    return -1;    
  }
  
  if(NULL == buf)
  {
    FFD_DEBUG_MSG("[FFD] ERROR - buf \n");
    return -1;    
  }

  if(count > RECEIVE_BUFFER_MAX_SIZE)
  {
    FFD_DEBUG_MSG("[FFD] ERROR - count \n");
    return -1;    
  }

  if(NULL == pos)
  {
    FFD_DEBUG_MSG("[FFD] ERROR - pos \n");
    return -1;    
  }

  memset(receive_buf, 0, sizeof(receive_buf));

  /* Test log
  {
    int remainingcount = 0;
    int rcTest = 0;

    rcTest = felica_uart_ioctrl(&remainingcount);
    FFD_DEBUG_MSG("[FFD] before remainedcount : %d \n",remainingcount);
    if (rcTest) {
      FFD_DEBUG_MSG("[FFD] ERROR - felica_uart_ioctrl \n");
      return rcTest;
    }
  
    if(4095 == remainedcount)
    {
      int i = 0;
      int testreadcount = 0;

      char *ptr = NULL;

      testreadcount = felica_uart_read(receive_buf,264);

      ptr = receive_buf;

      if(NULL != ptr)
      {
        FFD_DEBUG_MSG("===== TEST READ DATA =====\n");
        for(i=0; i<testreadcount; i++)
        {
          FFD_DEBUG_MSG(" %02x", *ptr++);
          if(0 == (i+1)%10)
          {
            FFD_DEBUG_MSG("\n");
          }
        }
        FFD_DEBUG_MSG("\n");
      }
    }
    
  }*/


  /* Copy UART receive data to receive buffer */
  mutex_lock(&ffd_mutex);
  readcount = ffu_uart_read(receive_buf,count);
  mutex_unlock(&ffd_mutex);

  if(0 >= readcount)
  {
    FFD_DEBUG_MSG("[FFD] ERROR - No data in data buffer \n");
    return 0;
  }

  /* you gotta fuck yourself, modyfing this line */
  //mdelay(5);

  /* Test log  
  {
    int remainingcount = 0;
    int rcTest = 0;

    rcTest = felica_uart_ioctrl(&remainingcount);
    FFD_DEBUG_MSG("[FFD] remaining count : %d \n",remainingcount);
    if (rcTest) {
      FFD_DEBUG_MSG("[FFD] ERROR - felica_uart_ioctrl \n");
      return rcTest;
    }
  }
  */

/* Display low data for debugging */
#ifdef RXTX_LOG_ENABLE
  {
    int i = 0;
    char *ptr = NULL;
    
    ptr = receive_buf;
    if(NULL != ptr)
    {
      FFD_DEBUG_MSG("===== READ FELICA LOW DATA =====\n");
      for(i=0; i<count; i++)
      {
        FFD_DEBUG_MSG(" %02x", *ptr++);
        if(0 == (i+1)%10)
        {
          FFD_DEBUG_MSG("\n");
        }
      }
      FFD_DEBUG_MSG("\n");
    }
  }
#endif
  /* Copy receive buffer to user memory */
  rc = copy_to_user(buf, receive_buf, count);

  if (rc) {
    FFD_DEBUG_MSG("[FFD] ERROR - copy_to_user \n");
    return rc;
  }

  #ifdef FFD_DEBUG_LOW
  FFD_DEBUG_MSG("[FFD] ffd_read - end \n");
  #endif

  return readcount;
}

/*
* Description :
* Input : 
* Output :
*/
static ssize_t ffd_write(struct file *fp, const char *buf, size_t count, loff_t *f_pos)
{
  int rc = 0;
  int writecount = 0;

  #ifdef FFD_DEBUG_LOW
  FFD_DEBUG_MSG("[FFD] ffd_write - start \n");
  #endif
 
  /* Check input parameters */
  if(NULL == fp)
  {
    FFD_DEBUG_MSG("[FFD] ERROR - fp \n");
    return -1;  
  }

  if(NULL == buf)
  {
    FFD_DEBUG_MSG("[FFD] ERROR - buf \n");
    return -1;  
  }

  if(count > TRANSMIT_BUFFER_MAX_SIZE)
  {
    FFD_DEBUG_MSG("[FFD] ERROR - count \n");
    return -1;  
  }
  
  /* Clear transmit buffer before using */
  memset(transmit_buf, 0, sizeof(transmit_buf));

  /* Copy user memory to kernel memory */
  rc = copy_from_user(transmit_buf, buf, count);
  if (rc) {
    FFD_DEBUG_MSG("[FFD] ERROR - copy_to_user \n");
    return rc;
  }

/* Display low data for debugging */
 #ifdef RXTX_LOG_ENABLE  
 {
    int i = 0;
    char *ptr = NULL;

    ptr = transmit_buf;
     
    if(NULL != ptr)
    {
      FFD_DEBUG_MSG("===== WRITE FELICA LOW DATA =====\n");
      for(i=0; i<count; i++)
      {
        FFD_DEBUG_MSG(" %02x", *ptr++);
        if(0 == (i+1)%10)
        {
          FFD_DEBUG_MSG("\n");
        }
      }
      FFD_DEBUG_MSG("\n");
    }   
  }
#endif

 
  /* Send transmit data to UART transmit buffer */
  mutex_lock(&ffd_mutex);
  writecount = ffu_uart_write(transmit_buf,count);
  mutex_unlock(&ffd_mutex);

  /* you gotta fuck yourself, modyfing this line */
  //mdelay(50);

  #ifdef FFD_DEBUG_LOW
  FFD_DEBUG_MSG("[FFD] writecount : %d \n",writecount);
  #endif

  return writecount;
}

/*
* Description :
* Input : 
* Output :
*/
static int ffd_release (struct inode *inode, struct file *fp)
{
  int rc = 0;

  /* Check input parameters */
  if(NULL == fp)
  {
    FFD_DEBUG_MSG("[FFD] ERROR - fp \n");
    return -1;  
  }
  /* FileInputStream and FileOutPutStream close felica
     Only one case has to be excuted */
  if(0 == isopen)
  {
    return 0;
  }
  else
  {
    isopen = 0;
  }

  #ifdef FFD_DEBUG_LOW
  FFD_DEBUG_MSG("[FFD] ffd_release - start \n");
  #endif

  rc = ffu_uart_close();
  if(rc)
  {
    FFD_DEBUG_MSG("[FFD] ERROR - open_hs_uart \n");
    return rc;
  }

  #ifdef FFD_DEBUG_LOW
  FFD_DEBUG_MSG("[FFD] ffd_release - end \n");
  #endif

  return 0;
}

/*
* Description : MFC calls this function using available method(int available()) of FileOutputStream class
* Input : None
* Output : Return the number of byte that can be read.
*/
static long ffd_ioctl (struct file *fp, unsigned int cmd, unsigned long arg)
{
  int numofreceiveddata = 0;
  int rc = 0;
  int *uarg = (int *)arg;


  #ifdef FFD_DEBUG_LOW
  FFD_DEBUG_MSG("[FFD] ffd_ioctl - start \n");
  #endif

  /* Check input parameters */
  if(NULL == fp)
  {
    FFD_DEBUG_MSG("[FFD] ERROR - fp \n");
    return -1;  
  }

  if(IOCTL_FFD_MAGIC != _IOC_TYPE(cmd)) 
  {
    FFD_DEBUG_MSG("[FFD] ERROR - IO cmd type \n");
    return -1;
  }

  if(IOCTL_FFD_CMD_AVAILABLE != _IOC_NR(cmd)) 
  {
    FFD_DEBUG_MSG("[FFD] ERROR - IO cmd number \n");
    return -1;
  }

  if(0 != _IOC_SIZE(cmd)) 
  {
    FFD_DEBUG_MSG("[FFD] ERROR - IO cmd size \n");
    return -1;
  }

  mutex_lock(&ffd_mutex);
  rc = ffu_uart_ioctrl(&numofreceiveddata);
  mutex_unlock(&ffd_mutex);

  if (rc) {
    FFD_DEBUG_MSG("[FFD] ERROR - ffd_uart_ioctrl \n");
    return rc;
  }

  /* you gotta fuck yourself, modyfing this line */
   //mdelay(20);
  
  rc = copy_to_user(uarg, &numofreceiveddata, sizeof(int));
  if(rc)
  {
    FFD_DEBUG_MSG("[FFD] ERROR - open_hs_uart \n");
    return rc;
  }

  #ifdef FFD_DEBUG_LOW
  FFD_DEBUG_MSG("[FFD] felica_ioctl - end \n");
  #endif

  return rc;
}

/*
* Description :
* Input : 
* Output :
*/
static int ffd_fsync(struct file *fp, int datasync)
{
  FFD_DEBUG_MSG("[FFD] ffd_fsync\n");

  // TODO: TRANSMIT DATA TO FELICA CHIP

  return 0;
}

static struct file_operations ffd_fops = 
{
  .owner    = THIS_MODULE,
  .open      = ffd_open,
  .read      = ffd_read,
  .write    = ffd_write,
  .unlocked_ioctl  = ffd_ioctl,
  .fsync    = ffd_fsync,
  .release  = ffd_release,
};

static struct miscdevice ffd_device = {
  .minor = 252,
  .name = FFD_NAME,
  .fops = &ffd_fops,
};

/*
* Description :
* Input : 
* Output :
*/
int ffd_init(void)
{
  int rc = 0;

  #ifdef FFD_DEBUG_LOW
  FFD_DEBUG_MSG("[FFD] ffd_init - start \n");
  #endif

  /* register the device file */
  rc = misc_register(&ffd_device);
  if(rc)
  {
    FFD_DEBUG_MSG("[FFD] ffd_init : error : can not register : %d\n", rc);
    return rc;
  }

  #ifdef FFD_DEBUG_LOW
  FFD_DEBUG_MSG("[FFD] ffd_init - end \n");
  #endif

  return 0;
}

/*
* Description :
* Input : 
* Output :
*/
void ffd_exit(void)
{
  #ifdef FFD_DEBUG_LOW
  FFD_DEBUG_MSG("[FFD] ffd_exit - start \n");
  #endif

  /* deregister the device file */
  misc_deregister(&ffd_device);

  #ifdef FFD_DEBUG_LOW
  FFD_DEBUG_MSG("[FFD] ffd_exit - end \n");
  #endif
}
