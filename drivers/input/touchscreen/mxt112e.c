/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
*/

#ifndef XXXDEBUG_LOGIC_TEST
#include <linux/module.h>
#include <linux/hrtimer.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <mach/hs_io_ctl_a.h>
#include <mach/pmic.h>
#include <linux/wakelock.h>

#include <mach/kcj_dev_info.h> 

#include <mach/proc_comm_kyocera.h>

#include <../../staging/android/select_class.h> 

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#ifdef REF_LOG
#include <linux/fs.h>
#include <linux/syscalls.h>

#include <linux/unistd.h>
#include <linux/fcntl.h>
#endif 
#endif

#undef  KEY_APP_SWITCH
#define KEY_APP_SWITCH 252

#define EVENT_MAX 4
#define EVENT_LIMIT EVENT_MAX

#define TP_NV_1_SIZE 128
#define TP_NV_2_SIZE 128

#define DISP_RESOLUTION_X 320
#define DISP_RESOLUTION_Y 480

#define TOUCHPANEL_SLEEP_STATE_RESUME     (0)
#define TOUCHPANEL_SLEEP_STATE_SUSPEND    (1)

#define TOUCHPANEL_MODE_STATE_NORMAL      (0)
#define TOUCHPANEL_MODE_STATE_PATENT      (1)

#define TOUCHPANEL_FILTER_STATE_NORMAL    (0)
#define TOUCHPANEL_FILTER_STATE_SEARCH    (1)

#define MESSAGE_PRCS  T5
#define COMMAND_PRCS  T6
#define CALI          2
#define DIAG          5
#define POWER_CFG     T7
#define MULTI_TOUCH   T9
#define BLEN          6
#define SELF_TEST     T25
#define DIAG_DEBUG    T37
#define NOISE_SUPPR   T48
#define CACFG_BYTE    2
#define CHARAGON_BIT  5
#define DISGC_BIT     1

enum { 
  T7=0, T9, T48, T47, T46, T42, T25, T8, T6,
  T5, T37,
  REG_SIZE,
};

const static unsigned char reg_t6[] ={0x55,0,0x55,0,0,0};
const static unsigned char reg_t7[] ={32,8,10};
const static unsigned char reg_t25[]={0,0,0xa0,0x73,0x84,0x4e};
const static unsigned char reg_t47[]={0,0,0,0,0,0,0,0,0,0};
#ifdef CONFIG_TARGET_PRODUCT_C5155
const static unsigned char reg_t8[] ={23,0,3,1,0,0,0,0,10,0};
const static unsigned char reg_t9[] ={143,0,0,14,8,0,32,46,1,3,0,20,2,64,4,10,25,0,0,0,0,0,1,1,45,45,234,53,231,85,50,10,0,0,3};
const static unsigned char reg_t42[]={3,25,30,25,88,0,0,0};
const static unsigned char reg_t46[]={0,0,16,32,0,0,1,0,0};
const static unsigned char reg_t48[]={3,128,96,0,0,0,0,0,10,15,0,0,0,6,6,0,0,100,4,32,28,0,20,4,0,28,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
#else 
const static unsigned char reg_t8[] ={23,0,1,1,0,0,0,0,10,0};
const static unsigned char reg_t9[] ={143,0,0,14,8,0,32,50,1,3,0,20,5,80,4,5,5,0,0,0,0,0,2,2,45,45,247,51,161,97,50,10,0,0,3};
const static unsigned char reg_t42[]={3,25,30,25,0,0,0,0};
const static unsigned char reg_t46[]={0,0,16,32,0,0,1,0,0};
const static unsigned char reg_t48[]={3,128,96,0,0,0,0,0,10,15,0,0,0,6,6,0,0,100,4,32,28,0,20,4,0,28,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
#endif 

const static u16 touch_field_x[] = {0,255,511,767,1023};
#ifdef CONFIG_TARGET_PRODUCT_C5155
const static u16 touch_field_y[] = {0,910,980,980,1024};
#else 
const static u16 touch_field_y[] = {0,935,950,950,1023};
#endif 

static DEFINE_MUTEX(tp_chg_mode_lock);

static int touch_probe(struct i2c_client *client,
              const struct i2c_device_id *id);

static struct workqueue_struct *touch_workqueue_event;
static struct workqueue_struct *touch_workqueue_debug;

struct ts_ID_Infromation {
  u8  FamilyID;
  u8  ValiantID;
  u8  Version;
  u8  Build;
  u8  xSize;
  u8  ySize;
  u8  elements;
};

struct ts_ObjectTable {
  u8  Type;
  u16 StartPosition;
  u8  Size;
  u8  Instance;
  u8  ReportID;
};

struct ts_mxt112e_Info {
  struct ts_ID_Infromation info;
  struct ts_ObjectTable table[REG_SIZE];

  u16     touch_start;
  u16     touch_end;
  
  short                process_info;
  short                process_result;

  int     irq;
  struct  i2c_client    *client;

  u16     field_x[5];
  u16     field_y[5];
};

struct ts_event {
  int  x;
  int  y;
  int  state;
  int  width;
};

struct mxt112e_touch {
  struct input_dev     *input;
  char                 phys[128];

  struct ts_mxt112e_Info panel_info;
  struct ts_event      event[EVENT_MAX];
  int                  sleep_state;
  int                  mode_state;
  int                  filter_state;
  
  int                  event_count;
  
  struct  work_struct  work_event;
  struct  work_struct  work_debug;
  struct  delayed_work work_recovery;
  struct  delayed_work work_calibration;
  
  int                  lock;
  
  char                 log_mode;
  
#ifdef REF_LOG
  struct timer_list    timer_ref_log;
#endif

  int                  noise_state;
  int                  usb_change_mode;
  
#ifdef CONFIG_HAS_EARLYSUSPEND
  struct early_suspend early_suspend;
#endif
};

static struct mxt112e_touch *ts_data;

static struct wake_lock touchpanel_wake_lock;

#define printlog if(ts_data->log_mode==1)printk

static unsigned int get_nv_log_mode(void){
  unsigned int ret;
  unsigned int i = TP_NV_2_SIZE/sizeof(unsigned int)-1;
  
  proc_comm_rpc_apps_to_modem(PROC_COMM_SUB_CMD_TP_READ_NV2, &i);
  i=i>>24;
  if(i==1 || i==2){
    printk("%s:valid touchpanel log\n",__func__);
    ret=i;
  }
  else {
    ret=0;
  }
  
  return ret;
}

static unsigned int get_nv_chargon_mode(void){
  unsigned int ret;
  unsigned int i = TP_NV_2_SIZE/sizeof(unsigned int)-1;
  
  proc_comm_rpc_apps_to_modem(PROC_COMM_SUB_CMD_TP_READ_NV2, &i);
  i=(i>>16) & 0xff;

  if(i>=1 && i<=6){
    printk("%s:valid touchpanel CHARGON USB toggle :value=%d\n",__func__, i);
    ret=i;
  }
  else {
    ret=0;
  }
  
  return ret;
}

static int touch_field_set(struct ts_mxt112e_Info *data){
  u8 nv_buf[64];
  int touch_field_nv_size = (ARRAY_SIZE(data->field_x) + ARRAY_SIZE(data->field_y))*sizeof(u16);
  int i;
  
  memset(nv_buf,0xff,ARRAY_SIZE(nv_buf));
  memcpy(data->field_x, touch_field_x, ARRAY_SIZE(touch_field_x)*sizeof(u16));
  memcpy(data->field_y, touch_field_y, ARRAY_SIZE(touch_field_y)*sizeof(u16));
  printlog("%s:field_x=%u,%u,%u,%u,%u\n",__func__,data->field_x[0],data->field_x[1],data->field_x[2],data->field_x[3],data->field_x[4]);
  printlog("%s:field_y=%u,%u,%u,%u,%u\n",__func__,data->field_y[0],data->field_y[1],data->field_y[2],data->field_y[3],data->field_y[4]);
  
  for(i=0;i<touch_field_nv_size/sizeof(unsigned int)+1;i++)
  {
    unsigned int tmp = i;
    proc_comm_rpc_apps_to_modem(PROC_COMM_SUB_CMD_TP_READ_NV, &tmp);
    memcpy(&nv_buf[i*sizeof(unsigned int)],&tmp,sizeof(unsigned int));
  }
  
  for(i=0; i<ARRAY_SIZE(touch_field_x); i++){
    int num = i*sizeof(u16);
    if(nv_buf[num]!=0xff||nv_buf[num+1]!=0xff){
      memcpy(&data->field_x[i],&nv_buf[num],sizeof(u16));
    }
  }
  
  for(i=0; i<ARRAY_SIZE(touch_field_y); i++){
    int num = (i+ARRAY_SIZE(touch_field_x))*sizeof(u16);
    if(nv_buf[num]!=0xff||nv_buf[num+1]!=0xff){
      memcpy(&data->field_y[i],&nv_buf[num],sizeof(u16));
    }
  }
  printlog("%s:field_x=%u,%u,%u,%u,%u\n",__func__,data->field_x[0],data->field_x[1],data->field_x[2],data->field_x[3],data->field_x[4]);
  printlog("%s:field_y=%u,%u,%u,%u,%u\n",__func__,data->field_y[0],data->field_y[1],data->field_y[2],data->field_y[3],data->field_y[4]);
  
  return 0;
}

static ssize_t touch_virtual_keys_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
  u16 *x = ts_data->panel_info.field_x;
  u16 *y = ts_data->panel_info.field_y;

  int key1_center_x = (x[1]+x[0])*DISP_RESOLUTION_X /(2*(x[4]-x[0]+1))+1;
  int key2_center_x = (x[2]+x[1])*DISP_RESOLUTION_X /(2*(x[4]-x[0]+1))+1;
  int key3_center_x = (x[3]+x[2])*DISP_RESOLUTION_X /(2*(x[4]-x[0]+1))+1;
  int key4_center_x = (x[4]+x[3])*DISP_RESOLUTION_X /(2*(x[4]-x[0]+1))+1;

  int key1_width    = (x[1]-x[0]+1)*DISP_RESOLUTION_X /(x[4]-x[0]+1);
  int key2_width    = (x[2]-x[1]+1)*DISP_RESOLUTION_X /(x[4]-x[0]+1);
  int key3_width    = (x[3]-x[2]+1)*DISP_RESOLUTION_X /(x[4]-x[0]+1);
  int key4_width    = (x[4]-x[3]+1)*DISP_RESOLUTION_X /(x[4]-x[0]+1);

  int keys_center_y = (y[4]+y[3])*DISP_RESOLUTION_Y /(2*(y[1]-y[0]+1))+1;
  int keys_height   = (y[4]-y[3]+1)*DISP_RESOLUTION_Y /(y[1]-y[0]+1)+1;

  return sprintf(buf,
      "0x01:%d:%d:%d:%d:%d:"
      "0x01:%d:%d:%d:%d:%d:"
      "0x01:%d:%d:%d:%d:%d:"
      "0x01:%d:%d:%d:%d:%d",
      KEY_BACK,       key1_center_x, keys_center_y, key1_width, keys_height,
      KEY_HOME,       key2_center_x, keys_center_y, key2_width, keys_height,
      KEY_APP_SWITCH, key3_center_x, keys_center_y, key3_width, keys_height,
      KEY_MENU,       key4_center_x, keys_center_y, key4_width, keys_height
    );
}

static struct kobj_attribute touch_virtual_keys_attr = {
  .attr = {
    .name = "virtualkeys.touchpanel",
    .mode = S_IRUGO,
  },
  .show = &touch_virtual_keys_show,
};

static struct attribute *touch_properties_attrs[] = {
	&touch_virtual_keys_attr.attr,
	NULL
};

static struct attribute_group touch_properties_attr_group = {
	.attrs = touch_properties_attrs,
};

static int touchpanel_i2c_read(struct i2c_client *client,u16 addr,u8 *buf,int size)
{
  u8 addr_buf[2];
  struct i2c_msg msg[2];

  addr_buf[0] = (u8)(addr&0x00ff);
  addr_buf[1] = (u8)((addr&0xff00)>>8);
  
  msg[0].addr  = client->addr;
  msg[0].flags = 0;
  msg[0].len   = 2;
  msg[0].buf   = addr_buf;
  
  msg[1].addr  = client->addr;
  msg[1].flags = I2C_M_RD;
  msg[1].len   = size;
  msg[1].buf   = buf;

  return i2c_transfer(client->adapter,msg,2);
}
#define TOUCHPANEL_I2C_WBUF_SIZE (64)
static int touchpanel_i2c_write(struct i2c_client *client,u16 addr,u8 *buf,int size)
{
  u8 w_buf[TOUCHPANEL_I2C_WBUF_SIZE+2];
  struct i2c_msg msg;

  if(size>TOUCHPANEL_I2C_WBUF_SIZE)
  {
    printk("%s: data size[%d] exceeds max size[%d]\n", __func__, size,TOUCHPANEL_I2C_WBUF_SIZE);
    return -1;
  }
  
  w_buf[0] = (u8)(addr&0x00ff);
  w_buf[1] = (u8)((addr&0xff00)>>8);
  
  memcpy(&w_buf[2],buf,size);
  
  msg.addr  = client->addr;
  msg.flags = 0;
  msg.len   = 2+size;
  msg.buf   = w_buf;
  
  return i2c_transfer(client->adapter,&msg,1);
}

#define TOUCHCNT_MAX (0xFFFFFFFF)

typedef enum {
  TOUCHCNT_READ_NV = 0,
  TOUCHCNT_WRITE_NV,
  TOUCHCNT_INC,
} touchcnt_ctl_cmd;

static DEFINE_MUTEX(tp_touchcnt_lock);

static unsigned int touchpanel_qm_touchcnt;
static bool is_valid_touchcnt;

static void touchpanel_touchcnt_read_nv(void)
{
  unsigned int subcmd = PROC_COMM_SUB_CMD_TP_TOUCHCNT_READ_NV;
  unsigned int data = 0;

  proc_comm_rpc_apps_to_modem(subcmd,&data);
 
  touchpanel_qm_touchcnt = data;
  if(TOUCHCNT_MAX != data)
  {
    is_valid_touchcnt = true;
  }
}

static void touchpanel_touchcnt_write_nv(void)
{
  unsigned int subcmd = PROC_COMM_SUB_CMD_TP_TOUCHCNT_WRITE_NV;
  unsigned int data = 0;

  if(is_valid_touchcnt == true)
  {
    data = touchpanel_qm_touchcnt;
    proc_comm_rpc_apps_to_modem(subcmd,&data);
  }
}

static void touchpanel_touchcnt_inc(void)
{
  if(TOUCHCNT_MAX > touchpanel_qm_touchcnt)
  {
    touchpanel_qm_touchcnt++;
  }
}

static void touchpanel_ctl_touchcnt(touchcnt_ctl_cmd cmd)
{
  if(!mutex_lock_interruptible(&tp_touchcnt_lock))
  {
    switch(cmd)
    {
    case TOUCHCNT_READ_NV:
      touchpanel_touchcnt_read_nv();
      break;
    case TOUCHCNT_WRITE_NV:
      touchpanel_touchcnt_write_nv();
      break;
    case TOUCHCNT_INC:
      touchpanel_touchcnt_inc();
      break;
    default:
      break;
    }
    mutex_unlock(&tp_touchcnt_lock);
  }
}

void touchpanel_touchcnt_write_nv_ex(void)
{
  touchpanel_ctl_touchcnt(TOUCHCNT_WRITE_NV);
}

#ifdef REF_LOG 

static int logsw=0;
static u8 file_buf[2048];
static u8 debug_buf[140];
static char ref_name[128];
static char delta_name[128];

void ref_log_sw(unsigned long sw)
{
  if(sw){
    printlog("%s:ref_log start\n", __func__);
    if(logsw==0) logsw=1;
  }
  else{
    printlog("%s:ref_log end\n", __func__);
    logsw=0;
  }
  mod_timer(&ts_data->timer_ref_log,jiffies + msecs_to_jiffies(1));
}

static void touch_ref_log(unsigned long data)
{
  queue_work(touch_workqueue_debug, &ts_data->work_debug);
}

static void touch_work_reflog(struct work_struct *work)
{
  int ret;
  char buf[16];
  struct file *file=0;
  char command=0xff;
  int wait=100;
  static struct file *ref_file, *delta_file;
  static int cntitem=0;

  printlog("%s:logsw=%d\n", __func__, logsw);
  if(logsw==0){
    if(ref_file)filp_close(ref_file, NULL);
    if(delta_file)filp_close(delta_file, NULL);
    ref_file=0;
    delta_file=0;
  }
  else if(logsw==1){
    int x, y;
    static int file_num=1;
    sprintf(ref_name,"/sdcard/tplog_ref%05d.csv",file_num);
    sprintf(delta_name,"/sdcard/tplog_delta%05d.csv",file_num);
    file_num++;

    file_buf[0] = 0;
    for(x=0; x<ts_data->panel_info.info.xSize; x++){
      for(y=0; y<ts_data->panel_info.info.ySize; y++){
        sprintf(buf, "X%dY%d,", x, y);
        strcat(file_buf, buf);
      }
    }       
    strcat(file_buf,"\n");

    ref_file    = filp_open(ref_name,   O_CREAT, S_IRWXU | S_IRWXG | S_IRWXO);
    delta_file  = filp_open(delta_name, O_CREAT, S_IRWXU | S_IRWXG | S_IRWXO);

    if (ref_file->f_op->write){
      ret = ref_file->f_op->write(ref_file, file_buf, strlen(file_buf), &(ref_file->f_pos));
    }else{
      ret = do_sync_write(ref_file, file_buf, strlen(file_buf), &(ref_file->f_pos));
    }

    if (delta_file->f_op->write){
      ret = delta_file->f_op->write(delta_file, file_buf, strlen(file_buf), &(delta_file->f_pos));
    }else{
      ret = do_sync_write(delta_file, file_buf, strlen(file_buf), &(delta_file->f_pos));
    }
    
    cntitem = 0;
    file_buf[0] = 0;
    command=0x10;
    logsw=2;
  }
  else if(logsw==2){
    debug_buf[0]=debug_buf[1]=0;
    touchpanel_i2c_read(ts_data->panel_info.client,
                        ts_data->panel_info.table[DIAG_DEBUG].StartPosition,
                        debug_buf, 
                        ts_data->panel_info.table[DIAG_DEBUG].Size);
                        
    printlog("%s:debug_buf[0-1]=%d, %d\n", __func__, debug_buf[0], debug_buf[1]);
    if(debug_buf[0]==0x10 || debug_buf[0]==0x11){ 
      if(debug_buf[0]==0x10){  
        int n;
        for(n=0;n<64 && cntitem<112;n++,cntitem++){
          short ref0=(short)(debug_buf[n*2+2]|(debug_buf[n*2+1+2]<<8));
          sprintf(buf,"%d,",ref0);
          strcat(file_buf,buf);
        }
      }
      else if(debug_buf[0]==0x11){ 
        int n;
        for(n=0;n<64 && cntitem<112;n++,cntitem++){
          sprintf(buf,"%d,",debug_buf[n*2+2]|(debug_buf[n*2+1+2]<<8));
          strcat(file_buf,buf);
        }
      }
      
      if(debug_buf[1]==1){
        if(debug_buf[0]==0x11) {
          file = ref_file;
          command=0x10;
        }
        else if(debug_buf[0]==0x10) {
          file = delta_file;
          command=0x11;
          wait = 2000;
        }
        
        file_buf[strlen(file_buf)-1]=0;
        strcat(file_buf,"\n");
        if (file->f_op->write){
          ret = file->f_op->write(file, file_buf, strlen(file_buf), &(file->f_pos));
        }
        else{
          ret = do_sync_write(file, file_buf, strlen(file_buf), &(file->f_pos));
        }
        cntitem = 0;
        file_buf[0] = 0;
      }
      else{
        command=0x01; 
      }
    }
  }

  if(logsw>=1){
    if(command!=0xff){
      printlog("%s: write command=%d\n", __func__, command);
      touchpanel_i2c_write( ts_data->panel_info.client,
                            ts_data->panel_info.table[COMMAND_PRCS].StartPosition+DIAG,
                            &command, 
                            1);
    }                              
    mod_timer(&ts_data->timer_ref_log,jiffies + msecs_to_jiffies(wait));
  }
}
#endif

static void touch_work_debug(struct work_struct *work)
{
#ifdef REF_LOG
  if(logsw)touch_work_reflog(0);
#endif
  printlog("********* touch_setcfg3\n");
  touchpanel_i2c_write(ts_data->panel_info.client,
                       ts_data->panel_info.table[POWER_CFG].StartPosition,
                       (u8*)reg_t7,
                       ts_data->panel_info.table[POWER_CFG].Size);
}

static void touch_work_calibration(struct work_struct *work)
{
  char buf=0x55;
  printlog("%s\n", __func__);
  touchpanel_i2c_write( ts_data->panel_info.client,
                        ts_data->panel_info.table[COMMAND_PRCS].StartPosition+CALI,
                        &buf,
                        1);
}

static void touch_change_state(int new_sleep, int new_mode, int new_filter)
{
  int old_status,new_status;
  static u8 power_cfg_buf[3];
  
  u8 buf[3];

  cancel_delayed_work(&ts_data->work_recovery);

  if((ts_data->sleep_state == TOUCHPANEL_SLEEP_STATE_SUSPEND) 
  && (ts_data->mode_state  == TOUCHPANEL_MODE_STATE_NORMAL)
  && (ts_data->filter_state== TOUCHPANEL_FILTER_STATE_NORMAL))
  {
    old_status = 0;
  }
  else
  {
    old_status = 1;
  } 
  
  if((new_sleep == TOUCHPANEL_SLEEP_STATE_SUSPEND) 
  && (new_mode  == TOUCHPANEL_MODE_STATE_NORMAL)
  && (new_filter== TOUCHPANEL_FILTER_STATE_NORMAL))
  {
    new_status = 0;
  }
  else
  {
    new_status = 1;
  }

  if(old_status != new_status)
  {
    cancel_delayed_work(&ts_data->work_calibration);

    if(new_status == 1)
    {
      printlog("%s: touchpanel active\n", __func__);
      touchpanel_i2c_write( ts_data->panel_info.client,
                            ts_data->panel_info.table[POWER_CFG].StartPosition,
                            (power_cfg_buf[0]) ? power_cfg_buf : (u8*)reg_t7,
                            3);

      buf[0]=0x55;
      touchpanel_i2c_write( ts_data->panel_info.client,
                            ts_data->panel_info.table[COMMAND_PRCS].StartPosition+CALI,
                            buf,
                            1);
                            
      queue_delayed_work(touch_workqueue_event, &ts_data->work_recovery, msecs_to_jiffies(1000));
    }
    else
    {
      printlog("%s: touchpanel sleep\n", __func__);
      touchpanel_i2c_read(  ts_data->panel_info.client,
                            ts_data->panel_info.table[POWER_CFG].StartPosition,
                            power_cfg_buf,
                            3);

      buf[0]=0;
      buf[1]=0;
      buf[2]=0;
      touchpanel_i2c_write( ts_data->panel_info.client,
                            ts_data->panel_info.table[POWER_CFG].StartPosition,
                            buf,
                            3);

      mdelay(50);
    }
  }
  ts_data->sleep_state = new_sleep;
  ts_data->mode_state = new_mode;
  ts_data->filter_state = new_filter;
}

static void touch_release_event(void)
{
  int n;
  printlog("%s: \n", __func__);
  for(n=0; n<EVENT_MAX; n++){
    if(ts_data->event[n].state==1){
      ts_data->event[n].state=0;
      ts_data->event[n].x=9999;
      ts_data->event[n].y=9999;
    }
  }
}

static void touch_send_event(unsigned long data)
{
  struct input_dev *input = ts_data->input;
  int n=0;
  int count=0;
  static int androidkey_flag = 1;
  
  printlog("%s: start\n", __func__);

  for(n=0; n<EVENT_MAX; n++){
    if(ts_data->event[n].state!=-1){
      if(ts_data->event[n].y <= ts_data->panel_info.field_y[1]) androidkey_flag=0;
    }
  }

  for(n=0; n<EVENT_MAX; n++){
    if(ts_data->event[n].state!=-1){
      int push;
      
      int position_x = (ts_data->event[n].x==1023) ? 1023 : ts_data->event[n].x+1;
      int position_y = (ts_data->event[n].y==1023) ? 1023 : ts_data->event[n].y+1;
      
      if(ts_data->event[n].state){
        count++;
        printlog("%s: Push::x:%d::y:%d\n", __func__, position_x, position_y);
        push = 255;
        
        if(!androidkey_flag && ts_data->event[n].y>ts_data->panel_info.field_y[1]){
          push=0;
          position_x = 9999;
          position_y = 9999;
        }

      }else{
        printlog("%s: Released\n", __func__);
        push = 0;
        ts_data->event[n].state=-1;

        touchpanel_ctl_touchcnt(TOUCHCNT_INC);
      }

      input_report_abs(input, ABS_MT_PRESSURE,push);
      input_report_abs(input, ABS_MT_POSITION_X, position_x);
      input_report_abs(input, ABS_MT_POSITION_Y, position_y);
      input_report_abs(input, ABS_MT_TOUCH_MAJOR,ts_data->event[n].width<<2);
      input_mt_sync(input);
      printlog("%s: input_mt_sync\n", __func__);
    }
  }
  
  input_report_key(input, BTN_TOUCH, 1);

  printlog("%s: count:%d\n", __func__, count);
  printlog("%s: input_sync\n", __func__);
  input_sync(input);

  if(ts_data->event_count==0 && count>0){ 
    kcj_dev_info_update_touch_panel(1);
  }
  else if(ts_data->event_count>0 && count==0){
    kcj_dev_info_update_touch_panel(0);
  }
  
  ts_data->event_count = count;

  if(count==0) {
    printlog("%s: input_mt_sync&input_sync\n", __func__);
    input_report_key(input, BTN_TOUCH, 0);
    input_mt_sync(input);
    input_sync(input);
    androidkey_flag=1;
  }
  ts_data->lock = 0;
  printlog("%s: end\n", __func__);
}

static int touch_read_values(struct ts_mxt112e_Info *tsc)
{
  u8  buf[16];
  int ret = 1;
  int release_touch = 0;

  printlog("%s: start\n", __func__);

  touchpanel_i2c_read(tsc->client,
                      tsc->table[MESSAGE_PRCS].StartPosition,
                      buf,
                      8);
  printlog("**** Message ID %d 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
           buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6]);
  
  if(buf[0] >= tsc->touch_start && buf[0] <= tsc->touch_end){
    int id = buf[0]-tsc->touch_start;
    int x, y, state;
    
    x=(buf[2]<<2)|(buf[4]>>6);
    y=(buf[3]<<2)|((buf[4]&0x0f)>>2);
    
    printlog("%s: x:%d y:%d\n", __func__, x, y);
    if(ts_data->log_mode==2) printk("%s: x:%d y:%d\n", __func__, x, y);
    
    ts_data->event[id].x=x;
    ts_data->event[id].y=y;
    ts_data->event[id].width=buf[5];

    state=(buf[1]>>7)&0x01;
    if(state && ts_data->event[id].state!=1 && ts_data->event_count>=EVENT_LIMIT){
      printlog("%s: touch_num limit over\n", __func__);
      ts_data->event[id].state = -1;
    }
    else {
      ts_data->event[id].state=state;
    }
    
    ret = 0;
  } 
  else if(buf[0] == 1){
    if(buf[1]&0x80) {
      printk("%s: touchpanel reset\n", __func__);
      release_touch = 1;
      ret = 0;
    }
    else if(buf[1]&0x10){
      printk("%s: touchpanel calibration\n", __func__);
      if(ts_data->panel_info.process_result==-1){
        ts_data->panel_info.process_result=1;
        ret = 1;
      }
      else {
        release_touch = 1;
        ret = 0;
      }
    }
  }
  else if(buf[0] == 6) {
    if(ts_data->panel_info.process_result==-1){
      ts_data->panel_info.process_result=buf[1];
    }

    ret = 1;
  }
  else if(buf[0] == 7) {
    if(buf[1]&0x1) {
      printk("%s: suppression Active\n", __func__);
      
      if(ts_data->mode_state==TOUCHPANEL_MODE_STATE_NORMAL){
        queue_delayed_work(touch_workqueue_event, &ts_data->work_calibration, msecs_to_jiffies(500));
      }
      ret = 1;
    }
  }
  else if(buf[0] == 9) {
    static int frequency_search_flag=0;
    
    printlog("%s: noise_fileter_status=%d\n", __func__, buf[4]);
    ts_data->noise_state = buf[4];
    ret = 1;
    
    if(frequency_search_flag && ts_data->noise_state>1){
      if(!mutex_lock_interruptible(&tp_chg_mode_lock)){
        printlog("%s: TOUCHPANEL_FILTER_STATE_NORMAL\n", __func__);
        touch_change_state(ts_data->sleep_state,ts_data->mode_state,TOUCHPANEL_FILTER_STATE_NORMAL);

        mutex_unlock(&tp_chg_mode_lock);
      }
      frequency_search_flag=0;
    }

    if(ts_data->noise_state==1) frequency_search_flag=1;
  }
  
  if(release_touch) touch_release_event();

  return ret;
}

static irqreturn_t touch_irq(int irq, void *handle)
{
  disable_irq_nosync(ts_data->panel_info.irq);

  cancel_delayed_work(&ts_data->work_recovery);
  queue_work(touch_workqueue_event, &ts_data->work_event);

  wake_lock_timeout(&touchpanel_wake_lock, 2 * HZ);
 
  return IRQ_HANDLED;
}
static void touch_work_event(struct work_struct *work)
{
  printlog("%s: touch_irq\n", __func__);
  if(!touch_read_values(&ts_data->panel_info))touch_send_event(0);
  
  enable_irq(ts_data->panel_info.irq);
}

static void touch_work_recovery(struct work_struct *work)
{
  u8 buf[2];
  touchpanel_i2c_read(ts_data->panel_info.client,
                      ts_data->panel_info.table[MESSAGE_PRCS].StartPosition,
                      buf,
                      2);
}

static int touch_remove(struct i2c_client *client)
{
  free_irq(ts_data->panel_info.irq, ts_data);
#ifdef CONFIG_HAS_EARLYSUSPEND
  unregister_early_suspend(&ts_data->early_suspend);
#endif
  input_unregister_device(ts_data->input);
  kfree(ts_data);

  return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void touch_early_suspend(struct early_suspend *h)
{
#ifndef REF_LOG
  printlog("%s: \n", __func__);
  mutex_lock(&tp_chg_mode_lock);
  touch_change_state(TOUCHPANEL_SLEEP_STATE_SUSPEND,ts_data->mode_state,ts_data->filter_state);
  mutex_unlock(&tp_chg_mode_lock);

#endif 
}

static void touch_early_resume(struct early_suspend *h)
{
#ifndef REF_LOG
  printlog("%s: \n", __func__);

  mutex_lock(&tp_chg_mode_lock);
  touch_change_state(TOUCHPANEL_SLEEP_STATE_RESUME,ts_data->mode_state,ts_data->filter_state);
  mutex_unlock(&tp_chg_mode_lock);

#endif 
}
#endif 

static int touch_suspend(struct i2c_client *client, pm_message_t state)
{
  printlog("%s: \n", __func__);

  return 0;
}

static int touch_resume(struct i2c_client *client)
{

  printlog("%s: \n", __func__);

  return 0;
}

static struct i2c_device_id touch_idtable[] = {
  { "touchpanel", 0 },
  { }
};

MODULE_DEVICE_TABLE(i2c, touch_idtable);

static struct i2c_driver touch_driver = {
  .driver = {
    .owner = THIS_MODULE,
    .name  = "touchpanel",
  },
  .id_table = touch_idtable,
  .probe    = touch_probe,
  .remove   = __devexit_p(touch_remove),
  .suspend  = touch_suspend,
  .resume   = touch_resume,
};

static int touch_init_device(struct ts_mxt112e_Info *data)
{
  u8 readbuf[132];
  u8  buf[132];
  u16 addr;

  int cnt=0;
  int max_report_id = 0;
  int ret=0;

  memset(buf,0,128);
  memset(readbuf,0,128);

  addr = 0;
  ret = touchpanel_i2c_read(data->client, addr, readbuf, 7);
  printlog("******** touch_init_device %x\n",ret);
  
  data->info.FamilyID  = readbuf[0];
  data->info.ValiantID = readbuf[1];
  data->info.Version   = readbuf[2];
  data->info.Build     = readbuf[3];
  data->info.xSize     = readbuf[4];
  data->info.ySize     = readbuf[5];
  data->info.elements  = readbuf[6];

  printlog("****FamilyID  : 0x%02x\n",data->info.FamilyID);
  printlog("****ValiantID : 0x%02x\n",data->info.ValiantID);
  printlog("****Version   : 0x%02x\n",data->info.Version);
  printlog("****Build     : 0x%02x\n",data->info.Build);
  printlog("****xSize     : %d\n",data->info.xSize);
  printlog("****ySize     : %d\n",data->info.ySize);
  printlog("****elements  : %d\n",data->info.elements);

  printlog("****Object Table Start\n");

  for (cnt = 1; cnt <= data->info.elements; cnt++){
    struct ts_ObjectTable *table=0;
    u8 type, size, instance, reportID;
    u16 startPosition;
    
    addr = cnt*6+1;
    touchpanel_i2c_read(data->client, addr, readbuf, 6);

    type          =readbuf[0];
    startPosition =readbuf[1]|(readbuf[2]<<8);
    size          =readbuf[3]+1;
    instance      =readbuf[4];
    reportID      =readbuf[5];

    printlog("\n****StartPosition %d\n",startPosition);
    printlog("****Size %d\n",size);
    printlog("****Instance %d\n",instance);
    printlog("****ReportID %d\n",reportID);
    printlog("****Type [%d]\n",type);
    
    switch(type){
    case 5:
      printlog("GEN_MESSAGEPROCESSOR\n");
      table=&ts_data->panel_info.table[T5];
      break;

    case 6:
      printlog("GEN_COMMANDPROCESSOR\n");
      table=&ts_data->panel_info.table[T6];
      break;

    case 7:
      printlog("GEN_POWERCONFIG\n");
      table=&ts_data->panel_info.table[T7];
      break;

    case 8:
      printlog("GEN_ACQUISITIONCONFIG\n");
      table=&ts_data->panel_info.table[T8];
      break;

    case 9:
      printlog("TOUCH_MULTITOUCHSCREEN\n");
      table=&ts_data->panel_info.table[T9];
      data->touch_start = max_report_id+1;
      data->touch_end = max_report_id + reportID;
      break;

    case 25:
      printlog("SPT_SELFTEST\n");
      table=&ts_data->panel_info.table[T25];
      break;

    case 37:
      printlog("DEBUG_DIAGNOSTIC\n");
      table=&ts_data->panel_info.table[T37];
      break;

    case 42:
      printlog("PROCI_TOUCHSUPPRESSION\n");
      table=&ts_data->panel_info.table[T42];
      break;

    case 46:
      printlog("SPT_CTECONFIG\n");
      table=&ts_data->panel_info.table[T46];
      break;

    case 47:
      printlog("PROCI_STYLUS\n");
      table=&ts_data->panel_info.table[T47];
      break;

    case 48:
      printlog("PROCG_NOISESUPPRESSION\n");
      table=&ts_data->panel_info.table[T48];
      break;

    default:
      printlog("not setting table t%d\n", type);
      break;     
    }      
    
    if(table!=0){
      table->Type         =type;
      table->StartPosition=startPosition;
      table->Size         =size;
      table->Instance     =instance;
      table->ReportID     =reportID;
    }
    
    max_report_id += reportID;

    if(reportID == 1){
      printlog("****Message ID : %d\n",max_report_id);
    }else if(reportID > 1){
      printlog("****Message ID : %d - %d\n",max_report_id - reportID+1,max_report_id);
    }else{
      printlog("****Not Message ID\n");
    }
  }
  printlog("****Object Table End\n");
  
  return 0;
}

static void touch_setcfg_init(struct ts_mxt112e_Info *data)
{
  u8  nv_buf[256];
  int i, posi=0;
  int update_reg=0;
  
  struct {
    struct ts_ObjectTable *table;
    const u8 *data;
    u8 nv_flag;
  } reg_set_data[] = {
    [T6]  ={ &ts_data->panel_info.table[T6],  reg_t6 , 0 },
    [T7]  ={ &ts_data->panel_info.table[T7],  reg_t7 , 1 },
    [T8]  ={ &ts_data->panel_info.table[T8],  reg_t8 , 1 },
    [T9]  ={ &ts_data->panel_info.table[T9],  reg_t9 , 1 },
    [T25] ={ &ts_data->panel_info.table[T25], reg_t25, 0 },
    [T42] ={ &ts_data->panel_info.table[T42], reg_t42, 1 },
    [T46] ={ &ts_data->panel_info.table[T46], reg_t46, 1 },
    [T47] ={ &ts_data->panel_info.table[T47], reg_t47, 1 },
    [T48] ={ &ts_data->panel_info.table[T48], reg_t48, 1 },
  };
  
  memset(nv_buf,0xff,ARRAY_SIZE(nv_buf));
  for(i=0;i<TP_NV_1_SIZE/sizeof(unsigned int);i++)
  {
    unsigned int tmp = i;
    proc_comm_rpc_apps_to_modem(PROC_COMM_SUB_CMD_TP_READ_NV, &tmp);
    memcpy(&nv_buf[i*sizeof(unsigned int)],&tmp,sizeof(unsigned int));
    printlog("%s:TP_NV_1: i=%d :%d, %d, %d, %d\n", __func__, 4*i, nv_buf[4*i], nv_buf[4*i+1], nv_buf[4*i+2], nv_buf[4*i+3]);
  }
  for(i=0;i<TP_NV_2_SIZE/sizeof(unsigned int);i++)
  {
    unsigned int tmp = i;
    proc_comm_rpc_apps_to_modem(PROC_COMM_SUB_CMD_TP_READ_NV2, &tmp);
    memcpy(&nv_buf[TP_NV_1_SIZE+i*sizeof(unsigned int)],&tmp,sizeof(unsigned int));
    printlog("%s:TP_NV_2: i=%d :%d, %d, %d, %d\n", __func__, TP_NV_1_SIZE+4*i, nv_buf[TP_NV_1_SIZE+4*i], nv_buf[TP_NV_1_SIZE+4*i+1], nv_buf[TP_NV_1_SIZE+4*i+2], nv_buf[TP_NV_1_SIZE+4*i+3]);
  }
  
  posi = (ARRAY_SIZE(ts_data->panel_info.field_x) + ARRAY_SIZE(ts_data->panel_info.field_y))*sizeof(u16);
  
  for(i=0; i<ARRAY_SIZE(reg_set_data); i++){
    u8  write_buf[128];
    u8  read_buf[128];
    
    memcpy(write_buf, reg_set_data[i].data, reg_set_data[i].table->Size);
    
    if(reg_set_data[i].nv_flag){
      int posi_cnt = posi;
      
      for(; posi_cnt-posi < reg_set_data[i].table->Size; posi_cnt++){
        int offset = posi_cnt-posi;
        int reg_16bit_flag = 0;
        
        if(i==T9){
           if(offset==18||offset==20) reg_16bit_flag=1;
        }else if(i==T46){
           if(offset==7) reg_16bit_flag=1;
        }else if(i==T48){
           if(offset==8||offset==20||offset==23||offset==25) reg_16bit_flag=1;
        }
        
        if(reg_16bit_flag){
          printlog("%s:cnt=%3d original:reg%2d=%3d,reg%2d=%3d, ", __func__, posi_cnt, offset, write_buf[offset], offset+1, write_buf[offset+1]);
          if(nv_buf[posi_cnt]!=0xff||nv_buf[posi_cnt+1]!=0xff){
            write_buf[offset] = nv_buf[posi_cnt];
            write_buf[offset+1] = nv_buf[posi_cnt+1];
          }
          printlog("nv:reg%2d=%3d,reg%2d=%3d\n", offset, write_buf[offset], offset+1, write_buf[offset+1]);
          posi_cnt++;
        }else{
          printlog("%s:cnt=%3d original:reg%2d=%3d, ", __func__, posi_cnt, offset, write_buf[offset]);
          if(nv_buf[posi_cnt]!=0xff) write_buf[offset] = nv_buf[posi_cnt];
          printlog("nv:reg%2d=%3d\n", offset, write_buf[offset]);
        }
      }
      
      posi += reg_set_data[i].table->Size;
    }
    
    touchpanel_i2c_read(  data->client, 
                          reg_set_data[i].table->StartPosition,
                          read_buf, 
                          reg_set_data[i].table->Size);
    
    if(memcmp(read_buf,write_buf,reg_set_data[i].table->Size) && i!=T6){
      update_reg=1;
      printk("%s:difference between read_buf and write_buf, reg_set_data[%d]\n", __func__, i);
    }
    
    if(i==T6){
      if(update_reg){
        write_buf[1]=0x55;
        printk("%s:update touchpanel non-volatile memory\n", __func__);
      }
      else {
        write_buf[1]=0;
      }
    }
    
    printlog("%s:write reg: id=%d, StartPosition=%d\n", __func__, i, reg_set_data[i].table->StartPosition);
    touchpanel_i2c_write( data->client,
                          reg_set_data[i].table->StartPosition,
                          write_buf,
                          reg_set_data[i].table->Size);
  }
}

static void process_check_enable(struct select_dev *dev, int value)
{
  char buf[2];
  char gain;
  short result; 
  int err;
  struct ts_mxt112e_Info *panel_info = &ts_data->panel_info;
  
  ts_data->panel_info.process_info = -1;

  ts_data->panel_info.process_result = -1;
  
  buf[0] = 0x03;
  buf[1] = 0x01;
  err=touchpanel_i2c_write( panel_info->client,
                            panel_info->table[SELF_TEST].StartPosition,
                            buf,
                            2);
  printk("**** %s: process_check IC power check waiting start.\n", __func__);
  msleep(100);
  result = ts_data->panel_info.process_result;
  printk("**** %s: process_check IC power check waiting end.\n", __func__);
  if (result == 0xfe) {
    printk("**** %s: process_check IC power check result: OK\n", __func__);
  }
  else if(result==-1){
    printk("**** %s: process_check IC power check result: TimeOut Err\n", __func__);
    ts_data->panel_info.process_info=6;
    return;
  } 
  else {
    printk("**** %s: process_check IC power check result: NG\n", __func__);
    printk("**** %s: process_check IC power check: status=%d\n", __func__, result );
    ts_data->panel_info.process_info=5;
    return;
  }

  ts_data->panel_info.process_result = -1;

  cancel_delayed_work(&ts_data->work_calibration);

  err=touchpanel_i2c_read(panel_info->client,
                          panel_info->table[MULTI_TOUCH].StartPosition+BLEN,
                          &gain,
                          1);
  printk("**** %s: process_check touchpanel_i2c_read=%d, read gain=%d\n", __func__, err, gain);

  buf[0] = 16;
  err=touchpanel_i2c_write( panel_info->client,
                            panel_info->table[MULTI_TOUCH].StartPosition+BLEN,
                            buf,
                            1);
  printk("**** %s: process_check touchpanel_i2c_write=%d, set gain=%d\n", __func__, err, buf[0]);

  err = 0;
  do {
    if (err++ >= 4) {
      printk("**** %s: process_check gain change err\n", __func__);
      touchpanel_i2c_write( panel_info->client,
                            panel_info->table[MULTI_TOUCH].StartPosition+BLEN,
                            &gain,
                            1);
      ts_data->panel_info.process_info = 4;
      return;
    }
    msleep(50);
  } while ( ts_data->panel_info.process_result != 1 );
  printk("**** %s: process_check gain change calibrate end\n", __func__);
  
  ts_data->panel_info.process_result = -1;
  
  printk("**** %s: process_check mode in\n", __func__);

  buf[0] = 0x03;
  buf[1] = 0xfe;
  err=touchpanel_i2c_write( panel_info->client,
                            panel_info->table[SELF_TEST].StartPosition,
                            buf,
                            2);
  printk("**** %s: process_check mode 100ms waiting start.\n", __func__);
  msleep(100);
  result = ts_data->panel_info.process_result;
  printk("**** %s: process_check mode 100ms waiting end.\n", __func__);
  if(result==0xfe){
    ts_data->panel_info.process_info=1;
    printk("**** %s: process_check result: OK\n", __func__);
  } 
  else if (result == -1) {
    ts_data->panel_info.process_info=3;
    printk("**** %s: process_check result: TimeOut Err\n", __func__);
  }
  else {
    ts_data->panel_info.process_info=2;
    printk("**** %s: process_check result: NG\n", __func__);
    printk("**** %s: process_check result: status=%d\n", __func__, result );
  }
  printk("**** %s: process_check mode process_info=%d\n", __func__, ts_data->panel_info.process_info);

  err=touchpanel_i2c_write( panel_info->client,
                            panel_info->table[MULTI_TOUCH].StartPosition+BLEN,
                            &gain, 
                            1);  
  printk("**** %s: touchpanel_i2c_write=%d, reset gain=%d\n", __func__, err, gain);
  printk("**** %s: process_check mode out\n", __func__);
  
  ts_data->panel_info.process_result=0;
}

static int process_check_state_get(struct select_dev *dev)
{
  printk("**** %s: process_info=%d\n", __func__, ts_data->panel_info.process_info);

  return ts_data->panel_info.process_info;
}

static struct select_dev process_check_mode = {
  .name = "kouteimode",
  .get_select = process_check_state_get,
  .enable = process_check_enable,
};

static void touch_patent_enable(struct select_dev *dev, int value)
{
  if(value!=0)
  {
    if(!mutex_lock_interruptible(&tp_chg_mode_lock))
    {
      touch_change_state(ts_data->sleep_state,TOUCHPANEL_MODE_STATE_PATENT,ts_data->filter_state);
      mutex_unlock(&tp_chg_mode_lock);
    }
  }
  else
  {
    if(!mutex_lock_interruptible(&tp_chg_mode_lock))
    {
      touch_change_state(ts_data->sleep_state,TOUCHPANEL_MODE_STATE_NORMAL,ts_data->filter_state);
      mutex_unlock(&tp_chg_mode_lock);
    }
  }
}

static int touch_patent_state_get(struct select_dev *dev)
{
  return ts_data->mode_state;
}

static struct select_dev patent_touchpanel = {
  .name = "touch_sensor",
  .get_select = touch_patent_state_get,
  .enable = touch_patent_enable,
};

static void touch_usb_connect(void){
  char state = 0x0;

  printlog("%s: \n", __func__);
  touchpanel_i2c_read(  ts_data->panel_info.client,
                        ts_data->panel_info.table[NOISE_SUPPR].StartPosition + CACFG_BYTE,
                        &state,
                        1);

  state |= 1 << DISGC_BIT;
  touchpanel_i2c_write( ts_data->panel_info.client,
                        ts_data->panel_info.table[NOISE_SUPPR].StartPosition + CACFG_BYTE,
                        &state,
                        1);
}

static void touch_usb_disconnect(void){
  char state = 0x0;

  if(!mutex_lock_interruptible(&tp_chg_mode_lock))
  {
    printlog("%s: TOUCHPANEL_FILTER_STATE_SEARCH\n", __func__);
    touch_change_state(ts_data->sleep_state,ts_data->mode_state,TOUCHPANEL_FILTER_STATE_SEARCH);
    mutex_unlock(&tp_chg_mode_lock);
  }
  
  printlog("%s: \n", __func__);
  touchpanel_i2c_read(  ts_data->panel_info.client,
                        ts_data->panel_info.table[NOISE_SUPPR].StartPosition + CACFG_BYTE,
                        &state,
                        1);
  printlog("%s: read state=%d\n", __func__, state);

  state ^= 1 << CHARAGON_BIT;
  touchpanel_i2c_write( ts_data->panel_info.client,
                        ts_data->panel_info.table[NOISE_SUPPR].StartPosition + CACFG_BYTE,
                        &state,
                        1);
  printlog("%s: write state=%d\n", __func__, state);

  msleep(40);

  state |= 1 << CHARAGON_BIT;
  state ^= 1 << DISGC_BIT;
  touchpanel_i2c_write( ts_data->panel_info.client,
                        ts_data->panel_info.table[NOISE_SUPPR].StartPosition + CACFG_BYTE,
                        &state,
                        1);
  printlog("%s: write state=%d\n", __func__, state);
}

static void touch_usbchange_enable(struct select_dev *dev, int state)
{
  static int last_state = 0;

  printlog("%s: state=%d, last_state=%d, noise=%d\n", __func__, state, last_state, ts_data->noise_state);

  if(state==1 && last_state==0){
    cancel_delayed_work(&ts_data->work_calibration);
    touch_usb_connect();
  }
  else if(state==0 && last_state==1){
    cancel_delayed_work(&ts_data->work_calibration);
    touch_usb_disconnect();
  }   
  last_state = state;
}

static int touch_usbchange_state_get(struct select_dev *dev)
{
  return ts_data->usb_change_mode;
}

static struct select_dev touch_usbchange = {
  .name = "tp_usbchange",
  .get_select = touch_usbchange_state_get,
  .enable = touch_usbchange_enable,
};

static void touch_burst_enable(struct select_dev *dev, int state)
{
  const u8 data_t9=0xD0;

  static u8 multi_touch_buf;
  static u8 step = 0;

  printlog("%s: \n", __func__);

  mutex_lock(&tp_chg_mode_lock);
  if(state==1 && step==0){
    if((ts_data->sleep_state == TOUCHPANEL_SLEEP_STATE_SUSPEND)
    && (ts_data->mode_state  == TOUCHPANEL_MODE_STATE_NORMAL)){
      printlog("%s: current is sleep\n", __func__);
      
      mutex_unlock(&tp_chg_mode_lock);
      return;
    }
    
    touchpanel_i2c_read(  ts_data->panel_info.client,
                          ts_data->panel_info.table[MULTI_TOUCH].StartPosition+13,
                          &multi_touch_buf,
                          1);
    
    touchpanel_i2c_write( ts_data->panel_info.client,
                          ts_data->panel_info.table[MULTI_TOUCH].StartPosition+13,
                          (u8 *)&data_t9,
                          1);
    step = 1;
  }
  else if(state!=1 && step==1){
    touchpanel_i2c_write( ts_data->panel_info.client,
                          ts_data->panel_info.table[MULTI_TOUCH].StartPosition+13,
                          &multi_touch_buf,
                          1);
    step = 0;
  }
  mutex_unlock(&tp_chg_mode_lock);
}

static int touch_burst_get_select(struct select_dev *dev)
{
  return 0;
}

static struct select_dev touch_burst = {
  .name       = "touch_burst",
  .enable     = touch_burst_enable,
  .get_select = touch_burst_get_select,
};

static int touch_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
  struct input_dev *input_dev;
  int err=0;
  int n;
  struct kobject *properties_kobj;
  int sysfs_create_group_err=0;
  
  printk("%s:\n",__func__);

  mdelay(60);

  ts_data = kzalloc(sizeof(struct mxt112e_touch), GFP_KERNEL);

  memset(ts_data,0,sizeof(struct mxt112e_touch));

  ts_data->log_mode=get_nv_log_mode();
  
  for(n=0; n<EVENT_MAX; n++){
    ts_data->event[n].state=-1;
  }
  
  input_dev = input_allocate_device();
  if (!ts_data || !input_dev) {
    err = -ENOMEM;
    goto err_free_mem;
  }

  ts_data->panel_info.client = client;
  client->driver = &touch_driver;
  i2c_set_clientdata(client, ts_data);

  ts_data->input = input_dev;
  
  if(touch_init_device(&ts_data->panel_info))goto err_free_mem;

#ifdef REF_LOG
  setup_timer(&ts_data->timer_ref_log, touch_ref_log,(unsigned long)ts_data);
#endif

  ts_data->usb_change_mode=get_nv_chargon_mode();
  ts_data->noise_state=0;

  INIT_WORK(&ts_data->work_event, touch_work_event);
  INIT_WORK(&ts_data->work_debug, touch_work_debug);
  INIT_DELAYED_WORK(&ts_data->work_recovery, touch_work_recovery);
  INIT_DELAYED_WORK(&ts_data->work_calibration, touch_work_calibration);

  ts_data->panel_info.irq = gpio_to_irq(GPIO_TP_INT_N);

  snprintf(ts_data->phys, sizeof(ts_data->phys),
    "%s/input/input0", dev_name(&client->dev));

  dev_info(&client->dev, "%s\n", ts_data->phys);

  input_set_drvdata(ts_data->input, ts_data);
  input_dev->name       = "touchpanel";
  input_dev->phys       = ts_data->phys;
  input_dev->id.bustype = BUS_I2C;
  input_dev->id.vendor  = 0x0001;
  input_dev->id.product = 0x0002;
  input_dev->id.version = 0x0100;
  input_dev->dev.parent = &client->dev;

  input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);

  set_bit(KEY_HOME, input_dev->keybit);
  set_bit(KEY_MENU, input_dev->keybit);
  set_bit(KEY_BACK, input_dev->keybit);
  set_bit(KEY_SEARCH, input_dev->keybit);
  
  touch_field_set(&ts_data->panel_info);  
  
  properties_kobj = kobject_create_and_add("board_properties", NULL);
  if (properties_kobj){
    sysfs_create_group_err = sysfs_create_group(properties_kobj, &touch_properties_attr_group);
    printlog("%s:sysfs_create_group:%d\n",__func__,sysfs_create_group_err);
  }else {
    printlog("%s:properties_kobj err\n",__func__);
  }

  set_bit(BTN_TOUCH, input_dev->keybit);
  input_dev->absbit[BIT_WORD(ABS_MT_POSITION_X)]  = BIT_MASK(ABS_MT_POSITION_X);
  input_dev->absbit[BIT_WORD(ABS_MT_POSITION_Y)]  = BIT_MASK(ABS_MT_POSITION_Y);
  input_dev->absbit[BIT_WORD(ABS_MT_PRESSURE)] = BIT_MASK(ABS_MT_PRESSURE);
  input_dev->absbit[BIT_WORD(ABS_MT_TOUCH_MAJOR)] = BIT_MASK(ABS_MT_TOUCH_MAJOR);

  input_set_abs_params(input_dev, ABS_MT_POSITION_X, ts_data->panel_info.field_x[0], ts_data->panel_info.field_x[4], 0, 0);
  input_set_abs_params(input_dev, ABS_MT_POSITION_Y, ts_data->panel_info.field_y[0], ts_data->panel_info.field_y[1], 0, 0);
  input_set_abs_params(input_dev, ABS_MT_PRESSURE,0, 255, 0, 0);
  input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,0, 255, 0, 0);
  err = input_register_device(input_dev);
  printlog("%s:input_register_device:%d\n",__func__,err);
  if (err)
    goto err_free_mem;

  select_dev_register(&process_check_mode);
  select_dev_register(&patent_touchpanel);
  select_dev_register(&touch_usbchange);
  select_dev_register(&touch_burst);

#ifdef CONFIG_HAS_EARLYSUSPEND
  ts_data->early_suspend.suspend = touch_early_suspend;
  ts_data->early_suspend.resume  = touch_early_resume;
  ts_data->early_suspend.level   = EARLY_SUSPEND_LEVEL_STOP_DRAWING;
  register_early_suspend(&ts_data->early_suspend);
#endif

  ts_data->sleep_state = TOUCHPANEL_SLEEP_STATE_RESUME;
  ts_data->mode_state  = TOUCHPANEL_MODE_STATE_NORMAL;
  ts_data->filter_state= TOUCHPANEL_FILTER_STATE_NORMAL;

  touch_setcfg_init(&ts_data->panel_info);

  wake_lock_init(&touchpanel_wake_lock, WAKE_LOCK_SUSPEND, "touchpanel");

  touchpanel_qm_touchcnt = 0;
  touchpanel_ctl_touchcnt(TOUCHCNT_READ_NV);

  err = request_irq(gpio_to_irq(GPIO_TP_INT_N), touch_irq, IRQF_TRIGGER_LOW, "touch0", ts_data);
  enable_irq_wake(gpio_to_irq(GPIO_TP_INT_N));

  return 0;

err_free_mem:
  input_free_device(input_dev);
#ifdef CONFIG_HAS_EARLYSUSPEND
  unregister_early_suspend(&ts_data->early_suspend);
#endif
  kfree(ts_data);
  return err;
}

static int __init touch_init(void)
{
  touch_workqueue_event = create_singlethread_workqueue("touch_workqueue_event");
  touch_workqueue_debug = create_singlethread_workqueue("touch_workqueue_debug");
  
  HS_VREG_L8_ON();
  
  gpio_tlmm_config(GPIO_CFG(GPIO_TP_INT_N,0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
  gpio_tlmm_config(GPIO_CFG(GPIO_TP_RST_N,0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

  gpio_set_value(GPIO_TP_RST_N, 0);
  mdelay(1);
  gpio_set_value(GPIO_TP_RST_N, 1);
  
  ts_data = (void*)0;
  
  return i2c_add_driver(&touch_driver);
}

static void __exit touch_exit(void)
{
  i2c_del_driver(&touch_driver);

  if (touch_workqueue_event)
    destroy_workqueue(touch_workqueue_event);

  if (touch_workqueue_debug)
    destroy_workqueue(touch_workqueue_debug);
}

#ifndef XXXDEBUG_LOGIC_TEST
module_init(touch_init);
module_exit(touch_exit);

MODULE_AUTHOR("Kyocera Corp.");
MODULE_DESCRIPTION("TouchScreen Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:mxt112e");
#endif

