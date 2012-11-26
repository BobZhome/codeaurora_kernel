/* This software is contributed or developed by KYOCERA Corporation. */
/* (C) 2012 KYOCERA Corporation                                      */

#include "msm_fb.h"
#include "mddihost.h"
#include "mddihosti.h"

#include <linux/gpio.h>
#include <mach/vreg.h>
#include <linux/regulator/consumer.h>
#include <linux/wakelock.h>
#include "dev_lcd_mddi_driver.h"
#include <mach/hs_io_ctl_a.h>
#include <linux/pwm.h>
#include <linux/mfd/pmic8058.h>
#include <linux/pmic8058-pwm.h>

#include <mach/kcj_dev_info.h>


#ifdef ENABLE_MDDI_MULTI_READ_WRITE
 #define LCD_CtrlWrite_Panel_M(addr, data_ptr, size)  mddi_host_register_multiwrite(addr, data_ptr, size, TRUE, NULL, MDDI_HOST_PRIM)
 #define LCD_CtrlRead_Panel_M( addr, data_ptr, size)  mddi_host_register_multiread( addr, data_ptr, size, TRUE, MDDI_HOST_PRIM)
#else  /* ENABLE_MDDI_MULTI_READ_WRITE */
 #define LCD_CtrlWrite_Panel_M(addr, data_ptr, size)
 #define LCD_CtrlRead_Panel_M( addr, data_ptr, size)
#endif /* ENABLE_MDDI_MULTI_READ_WRITE */

#define LCD_CtrlWrite_Panel_S(addr, data)  mddi_queue_register_write(addr,  data, TRUE, 0)
#define LCD_CtrlRead_Panel_S( addr, data)  mddi_queue_register_read( addr, &data, TRUE, 0)
#define MAX_BACKLIGHT_BRIGHTNESS 255

boolean dev_lcd_testmode_frex_check( void );
int oem_panel_read_power_mode( void );

static int g_device_lcd_state = TRUE;
int g_oem_dev_lcdbl_value = (MAX_BACKLIGHT_BRIGHTNESS / 2);
static int dev_lcd_frex_check = FALSE;
int  g_lcd_on_first_bl = TRUE;

extern int pwm_config_oem(struct pwm_device *pwm, int duty_us);
static struct pwm_device *bl_pwm;

struct regulator *l11;
static int dev_lcd_vreg_on( void )
{
  int rc = 0;

  l11 = regulator_get(NULL, "gp2");

  if (!l11) {
    pr_err("%s: VREG L11 get failed\n", __func__);
    return -1;
  }

  rc = regulator_set_voltage(l11, 2800000,2800000);
  if (rc) {
    pr_err("%s: VREG L11 set failed\n", __func__);
    goto fail_l11;
  }

  rc = regulator_enable(l11);
  if (rc) {
    pr_err("%s: VREG L11 enable failed\n", __func__);
    goto fail_l11;
  }

  return 0;

fail_l11:
  regulator_put(l11);
  l11 = NULL;
  return rc;
}

static void dev_lcd_vreg_off( void )
{
  if(l11){
    if(regulator_disable(l11))
    {
      printk("%s() Disable Error!!!\n", __func__);
    }
    regulator_put(l11);
    l11 = NULL;
  }
}

int dev_lcd_init_reset( void )
{
  msleep(1);
  gpio_direction_output(GPIO_LCD_RESET,1);
  msleep(2);
  gpio_direction_output(GPIO_LCD_RESET,0);
  msleep(2);
  gpio_direction_output(GPIO_LCD_RESET,1);
  msleep(10);
  printk(KERN_ERR "%s() called End\n", __func__);
  return(0);
}

static int dev_lcd_reset( void )
{
  msleep(1);
  gpio_direction_output(GPIO_LCD_RESET,0);
  msleep(1);
  gpio_direction_output(GPIO_LCD_RESET,1);
  msleep(10);
  printk(KERN_ERR "%s() called End\n", __func__);
  return(0);
}

/*********************************************************
                     Sleep禁止/解除
**********************************************************/
struct wake_lock lcd_wlock;
struct wake_lock lcd_wlock2;

#define DEV_LCD_WLOCK_INIT() wake_lock_init(&lcd_wlock , WAKE_LOCK_IDLE    , "lcd_wlock"); \
                             wake_lock_init(&lcd_wlock2, WAKE_LOCK_SUSPEND , "lcd_wlock2")
static void dev_lcd_wakelock_core( int onoff )
{
  static int lcd_wlock_flg = -1;
  if(onoff)
  {
    if(lcd_wlock_flg != TRUE)
    {
      wake_lock(&lcd_wlock);
      wake_lock(&lcd_wlock2);
      lcd_wlock_flg = TRUE;
    }
  }
  else
  {
    if(lcd_wlock_flg != FALSE)
    {
      wake_unlock(&lcd_wlock);
      wake_unlock(&lcd_wlock2);
      lcd_wlock_flg = FALSE;
    }
  }
}
int lcd_wlock_ctrl_flg = 0;

void oem_dev_lcd_wakelock( int type )
{
  lcd_wlock_ctrl_flg |= type;
  if(lcd_wlock_ctrl_flg)
  {
    dev_lcd_wakelock_core(TRUE);
  }
}

void oem_dev_lcd_wakeunlock( int type )
{
  lcd_wlock_ctrl_flg &= ~type;
  if(lcd_wlock_ctrl_flg == 0)
  {
    dev_lcd_wakelock_core(FALSE);
  }
}

void oem_dev_lcd_backlight_enable(void)
{
  if(oem_dev_lcd_connect_check() == FALSE)
  {
    printk(KERN_ERR "%s() Not LCD Connect No init!! \n", __func__);
  }
  else
  {
    gpio_direction_output(GPIO_LCD_BACKLIGHT_EN,1);
  }
}
void oem_dev_lcd_backlight_disable(void)
{
  pwm_disable(bl_pwm);
  gpio_direction_output(GPIO_LCD_BACKLIGHT_EN,0);
  msleep(200);
}

extern int pwm_config_oem(struct pwm_device *pwm, int duty_us);
static struct pwm_device *bl_pwm;

static void dev_lcdbl_pmic_init( void )
{
  bl_pwm = pwm_request(0, "dev_lcd_bl");
  if (bl_pwm == NULL || IS_ERR(bl_pwm)) {
    pr_err("%s pwm_request() failed\n", __func__);
    bl_pwm = NULL;
  }
  printk(KERN_INFO "DEV LCD: bl_pwm=%x LPG_chan=%d\n", (int) bl_pwm, (int)0);
}

static void dev_lcdbl_PWM( int pwm )
{
  if(oem_dev_lcd_connect_check() == FALSE)
  {
    printk(KERN_ERR "%s() Not LCD Connect No init!! \n", __func__);
    return;
  }

  oem_dev_lcd_wakelock(0x02);
  if (bl_pwm) {
    pwm_config_oem(bl_pwm, pwm);
    pwm_enable(bl_pwm);
  }
  oem_dev_lcd_wakeunlock(0x02);
}

void oem_dev_lcdbl_set(int lvl)
{
  static int pre_lvl = 0;
  if(lvl > MAX_BACKLIGHT_BRIGHTNESS)
  {
    lvl = MAX_BACKLIGHT_BRIGHTNESS;
  }
  else if (lvl < 0)
  {
    lvl = 0;
  }

  g_oem_dev_lcdbl_value = lvl;
  if(lvl <  20)
  {
    lvl = 511 - ((lvl * 67) / 20);
  }
  else
  {
    lvl = 475 - ((lvl * 359) / 235);
  }

  if((abs(g_oem_dev_lcdbl_value - pre_lvl) >= 10) || (g_oem_dev_lcdbl_value == 0)){
    printk(KERN_ERR "%s() And[%d]HW[%d]\n", __func__, g_oem_dev_lcdbl_value, lvl);
    pre_lvl = g_oem_dev_lcdbl_value;
  }
  if(oem_dev_lcd_status_get() == TRUE)
  {
    dev_lcdbl_PWM(lvl);

    kcj_dev_info_update_lcd_lv( (u32)g_oem_dev_lcdbl_value, 1 );

  }
}

void oem_dev_lcdbl_first_blon( void )
{
  if(g_lcd_on_first_bl == TRUE)
  {
    oem_dev_lcd_wakelock( 0x04 );
    msleep(55);

    if(g_device_lcd_state){
      oem_dev_lcdbl_set(g_oem_dev_lcdbl_value);
      oem_dev_lcd_backlight_enable();
    }else{
      printk("Not %s()!!!!!!! \n", __func__);
    }
    g_lcd_on_first_bl = FALSE;
    oem_dev_lcd_wakeunlock( 0x04 );
  }
}

uint32 gamma_A[24] = {
 0x00, 0x10, 0x19, 0x25, 0x33, 0x4B, 0x3C, 0x2C, 
 0x20, 0x18, 0x13, 0x10, 0x00, 0x10, 0x19, 0x25,
 0x33, 0x4B, 0x3C, 0x2C, 0x20, 0x18, 0x13, 0x10
};
uint32 gamma_B[24] = {
 0x00, 0x10, 0x19, 0x25, 0x33, 0x4B, 0x3C, 0x2C, 
 0x20, 0x18, 0x13, 0x10, 0x00, 0x10, 0x19, 0x25,
 0x33, 0x4B, 0x3C, 0x2C, 0x20, 0x18, 0x13, 0x10
};
uint32 gamma_C[24] = {
 0x00, 0x10, 0x19, 0x25, 0x33, 0x4B, 0x3C, 0x2C, 
 0x20, 0x18, 0x13, 0x10, 0x00, 0x10, 0x19, 0x25,
 0x33, 0x4B, 0x3C, 0x2C, 0x20, 0x18, 0x13, 0x10
};

enum{
  LCD_GAMMA_26,
  LCD_GAMMA_24,
  LCD_GAMMA_22,
  LCD_GAMMA_20,
  LCD_GAMMA_18,
};
int debug_lcd_gamma = LCD_GAMMA_22;
void dev_lcd_gamma_curve_set( int num )
{
  uint32 command;
  uint32 parameter[32] = {0};

  command             = 0xC8;
  memcpy(parameter, gamma_A, sizeof(gamma_A));
  LCD_CtrlWrite_Panel_M(command, parameter, 24); 
  command             = 0xC9;
  memcpy(parameter, gamma_B, sizeof(gamma_B));
  LCD_CtrlWrite_Panel_M(command, parameter, 24); 
  command             = 0xCA;
  memcpy(parameter, gamma_C, sizeof(gamma_C));
  LCD_CtrlWrite_Panel_M(command, parameter, 24); 
}
static int dev_lcd_resister_init_sequence(void)
{
  unsigned int command;
  unsigned int parameter[32];
  {
    dev_lcd_reset();
  }

  msleep(75);

  LCD_CtrlWrite_Panel_S(0x00, 0x00);
  LCD_CtrlWrite_Panel_S(0x10, 0x00);
  msleep(75);
  LCD_CtrlWrite_Panel_S(0xB0, 0x04);
  LCD_CtrlWrite_Panel_S(0x13, 0x00);
  LCD_CtrlWrite_Panel_S(0x35, 0x00);
  LCD_CtrlWrite_Panel_S(0x36, 0x40);
  LCD_CtrlWrite_Panel_S(0x3A, 0x66);
  command             = 0xB3;
  parameter[0]        =       0x02;
  parameter[1]        =       0x00;
  parameter[2]        =       0x00;
  parameter[3]        =       0x21;
  LCD_CtrlWrite_Panel_M(command, parameter, 4);
  LCD_CtrlWrite_Panel_S(0xB4, 0x00);
  LCD_CtrlWrite_Panel_S(0x11, 0x00);
  msleep(150);
  command             = 0xB7;
  parameter[0]        =       0x00;
  parameter[1]        =       0x00;
  parameter[2]        =       0x11;
  parameter[3]        =       0x25;
  LCD_CtrlWrite_Panel_M(command, parameter, 4); 
  command             = 0xC0;
  parameter[0]        =       0x02;
  parameter[1]        =       0xDF;
  parameter[2]        =       0x40;
  parameter[3]        =       0x13;
  parameter[4]        =       0x10;
  parameter[5]        =       0x13;
  parameter[6]        =       0x00;
  parameter[7]        =       0x44;
  LCD_CtrlWrite_Panel_M(command, parameter, 8); 
  command             = 0xC1;
  parameter[0]        =       0x07;
  parameter[1]        =       0x27;
  parameter[2]        =       0x05;
  parameter[3]        =       0x05;
  parameter[4]        =       0x50;
  LCD_CtrlWrite_Panel_M(command, parameter, 5); 
  command             = 0xC4;
  parameter[0]        =       0x30;
  parameter[1]        =       0x00;
  parameter[2]        =       0x03;
  parameter[3]        =       0x01;
  LCD_CtrlWrite_Panel_M(command, parameter, 4); 

  dev_lcd_gamma_curve_set(LCD_GAMMA_22);

  command             = 0xD0;
  parameter[0]        =       0xA9;
  parameter[1]        =       0x06;
  parameter[2]        =       0x08;
  parameter[3]        =       0x20;
  parameter[4]        =       0x35;
  parameter[5]        =       0x04;
  parameter[6]        =       0x01;
  parameter[7]        =       0x00;
  parameter[8]        =       0x08;
  parameter[9]        =       0x01;
  parameter[10]       =       0x00;
  parameter[11]       =       0x06;
  parameter[12]       =       0x01;
  parameter[13]       =       0x00;
  parameter[14]       =       0x00;
  parameter[15]       =       0x20;
  LCD_CtrlWrite_Panel_M(command, parameter, 16); 
  command             = 0xD1;
  parameter[0]        =       0x02;
  parameter[1]        =       0x2A;
  parameter[2]        =       0x1F;
  parameter[3]        =       0x3F;
  LCD_CtrlWrite_Panel_M(command, parameter, 4); 
  command             = 0x2A;
  parameter[0]        =       0x00;
  parameter[1]        =       0x00;
  parameter[2]        =       0x01;
  parameter[3]        =       0x3F;
  LCD_CtrlWrite_Panel_M(command, parameter, 4); 
  command             = 0x2B;
  parameter[0]        =       0x00;
  parameter[1]        =       0x00;
  parameter[2]        =       0x01;
  parameter[3]        =       0xDF;
  LCD_CtrlWrite_Panel_M(command, parameter, 4); 
  LCD_CtrlWrite_Panel_S(0x29, 0x00);
  msleep(75);
/*-------------------------------------------------------------*/
  return(0);
}

static int dev_lcd_disp_on(struct platform_device *pdev)
{
  struct msm_fb_data_type *mfd;    
  int ret;

  mfd = platform_get_drvdata(pdev);

  printk(KERN_ERR "%s() Start panel.id = %d img(%x)\n", __func__, mfd->panel.id, mfd->fb_imgType);

  if (!mfd){
    printk(KERN_ERR "%s() error msd", __func__);
    return -ENODEV;
  }
  if (mfd->key != MFD_KEY){
    printk(KERN_ERR "%s() error msd->key = %d\n", __func__, mfd->key);
    return -EINVAL;
  }

  if(oem_dev_lcd_connect_check() == FALSE)
  {
    printk(KERN_ERR "%s() Not LCD Connect No init!! \n", __func__);
    return(0);
  }
  if(g_device_lcd_state == TRUE)
  {
    printk(KERN_ERR "Status == LCD Power ON!!! \n");
    return(0);
  }


  oem_dev_lcd_wakelock(0x01);

  ret = dev_lcd_resister_init_sequence();
  if(ret){
    printk(KERN_ERR "%s(): dev_lcd_resister_init_sequence() Failed!!!!!!!\n", __func__);
  }
  mddi_host_write_pix_attr_reg(MDDI_DEFAULT_PRIM_PIX_ATTR);

  g_device_lcd_state = TRUE;

  printk(KERN_ERR "%s() End %x\n", __func__, ret);
 
  dev_lcd_frex_check = dev_lcd_testmode_frex_check();

  g_lcd_on_first_bl = TRUE;
  oem_dev_lcd_wakeunlock(0x01);

  kcj_dev_info_update_lcd_onoff(true);
  return 0;
}

void dev_mddi_host_hibernetion_check( void )
{
  mddi_host_type host_idx = MDDI_HOST_PRIM;
  uint32 status_reg;
  int cnter = 100;
  while(cnter){
    msleep(1);
    status_reg = mddi_host_reg_in(STAT);
    printk("%s(MDDI STS[%x])\n", __func__, status_reg);
    if(status_reg & 0x10){
      break;
    }
    cnter--;
  }
}

static int dev_lcd_disp_off(struct platform_device *pdev)
{
  struct msm_fb_data_type *mfd;    
  mfd = platform_get_drvdata(pdev);
  if (!mfd){
    printk(KERN_ERR "%s() error msd", __func__);
    return -ENODEV;
  }
  if (mfd->key != MFD_KEY){
    printk(KERN_ERR "%s() error msd->key = %d\n", __func__, mfd->key);
    return -EINVAL;
  }

  mdp_hw_vsync_clk_disable(mfd);

  if(oem_dev_lcd_connect_check() == FALSE)
  {
    printk(KERN_ERR "%s() Not LCD Connect No init!! \n", __func__);
    return(0);
  }
  if(g_device_lcd_state == FALSE)
  {
    printk(KERN_ERR "Status == LCD Power OFF!!! \n");
    return(0);
  }


  printk(KERN_ERR "%s() Start \n", __func__);
  oem_dev_lcd_wakelock(0x01);
  g_device_lcd_state = FALSE;
  dev_lcdbl_PWM(0);
  oem_dev_lcd_backlight_disable();

  LCD_CtrlWrite_Panel_S(0x10, 0x00);
  msleep(100);
  LCD_CtrlWrite_Panel_S(0xB0, 0x00);
  LCD_CtrlWrite_Panel_S(0xB1, 0x01);

  dev_mddi_host_hibernetion_check();

  oem_dev_lcd_wakeunlock(0x01);

  printk(KERN_ERR "%s() End \n", __func__);

  kcj_dev_info_update_lcd_onoff(false);

  return 0;
}

boolean lcd_is_shutdown = FALSE; 
void dev_lcd_shutdown( struct platform_device *pdev )
{
  if(oem_dev_lcd_connect_check() == FALSE)
  {
    printk(KERN_ERR "%s() Not LCD Connect No init!! \n", __func__);
    return;
  }

  if(g_device_lcd_state == FALSE)
  {
    printk(KERN_ERR "Status == LCD Power OFF!!! \n");
    return;
  }

  printk(KERN_ERR "%s() Start \n", __func__);

  oem_dev_lcd_wakelock(0x01);

  g_device_lcd_state = FALSE;
  dev_lcdbl_PWM(0);
  oem_dev_lcd_backlight_disable();

  LCD_CtrlWrite_Panel_S(0xB0, 0x00);
  LCD_CtrlWrite_Panel_S(0xB1, 0x01);
  LCD_CtrlWrite_Panel_S(0x28, 0x00);
  LCD_CtrlWrite_Panel_S(0x10, 0x00);

  lcd_is_shutdown = TRUE;

  dev_lcd_vreg_off();

  oem_dev_lcd_wakeunlock(0x01);

  printk(KERN_ERR "%s() End \n", __func__);
}

static void dev_lcd_set_backlight(struct msm_fb_data_type *mfd)
{
  oem_dev_lcdbl_set(mfd->bl_level);
}


static int dev_lcd_probe(struct platform_device *pdev)
{
  printk(KERN_ERR "%s() pdev->name=[%s]\n", __func__, pdev->name);
  msm_fb_add_device(pdev);
  return 0;
}

static struct platform_driver this_driver = {
  .probe    = dev_lcd_probe,
  .shutdown = dev_lcd_shutdown,
  .driver = {
    .name = "dev_lcd_hvga",
  },
};

static struct msm_fb_panel_data dev_lcd_panel_data = {
  .on  = dev_lcd_disp_on ,
  .off = dev_lcd_disp_off,
  .set_backlight  = dev_lcd_set_backlight,
  .set_vsync_notifier = NULL,
};

static struct platform_device this_device = {
  .name = "dev_lcd_hvga",
  .id   = 0,
  .dev  = {
    .platform_data = &dev_lcd_panel_data,
  }
};

static int __init dev_lcd_init(void)
{
  int ret;
  struct msm_panel_info *pinfo;

    printk(KERN_ERR "%s() \n", __func__);

  ret = platform_driver_register(&this_driver);
  if (!ret) {
    pinfo = &dev_lcd_panel_data.panel_info;
    pinfo->xres     = 320;
    pinfo->yres     = 480;
    pinfo->type     = MDDI_PANEL;
    pinfo->pdest    = DISPLAY_1;
    pinfo->mddi.vdopkt  = MDDI_DEFAULT_PRIM_PIX_ATTR;
    pinfo->wait_cycle   = 0;
    pinfo->bpp      = 18;
    pinfo->fb_num   = 2;
    pinfo->bl_max   = 0xFF;
    pinfo->bl_min   = 0x01;
    pinfo->clk_rate =  245760000 / 2;
    pinfo->clk_min  =  245760000 / 2;
    pinfo->clk_max  =  245760000 / 2;

    pinfo->lcd.vsync_enable     = TRUE;

    pinfo->lcd.refx100          = 6105;
    pinfo->lcd.v_back_porch     = 5;
    pinfo->lcd.v_front_porch    = 5;
    pinfo->lcd.v_pulse_width    = 0;
    pinfo->lcd.hw_vsync_mode    = TRUE;
    pinfo->lcd.vsync_notifier_period    = 0;

    ret = platform_device_register(&this_device);
    if (ret){
      platform_driver_unregister(&this_driver);
    }
  }

  gpio_tlmm_config(GPIO_CFG( GPIO_LCD_BACKLIGHT_EN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
  gpio_tlmm_config(GPIO_CFG( GPIO_LCD_RESET, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

  DEV_LCD_WLOCK_INIT();
  dev_lcd_wakelock_core( FALSE );
  dev_lcdbl_pmic_init();

  if(oem_dev_lcd_connect_check() == TRUE)
  {
    dev_lcd_vreg_on();
    oem_dev_lcd_wakelock(0x01);
    if(oem_panel_read_power_mode() == 0x1C){
      g_device_lcd_state = TRUE;
      LCD_CtrlWrite_Panel_S(0x3A, 0x66);
      dev_lcd_frex_check = dev_lcd_testmode_frex_check();
    }else{
      g_device_lcd_state = FALSE;
    }
    oem_dev_lcd_wakeunlock(0x01);
  }
  else
  {
    dev_lcd_vreg_on();
    dev_lcd_vreg_off();
    kcj_dev_info_update_lcd_onoff(false);
  }
  return ret;
}

module_init(dev_lcd_init);

#ifdef CONFIG_LCD_CONNECT_CHECK
static boolean dev_lcd_connect_check_proc( void )
{
  gpio_tlmm_config(GPIO_CFG( GPIO_LCD_CONNECT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);/* input pull-up */
  msleep(1);
  if(HS_GET_LCD_CONNECT())
  {
    return(TRUE);
  }
  else
  {
    return(FALSE);
  }
}
#endif /* CONFIG_LCD_CONNECT_CHECK */

boolean oem_dev_lcd_connect_check( void )
{
#ifdef CONFIG_LCD_CONNECT_CHECK
  static boolean oem_lcd_connect_status = FALSE;
  static boolean first_flg = TRUE;
  if(first_flg == TRUE)
  {
    oem_lcd_connect_status = dev_lcd_connect_check_proc();
    if(oem_lcd_connect_status == TRUE){/* connect */
      gpio_tlmm_config(GPIO_CFG( GPIO_LCD_CONNECT, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);/* input no-pull */
    }else{/* not connect */
      gpio_tlmm_config(GPIO_CFG( GPIO_LCD_CONNECT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);/* input pull-up */
    }
    printk(KERN_ERR "%s() LCD Connect[%x]GPIO[%x]\n", __func__, oem_lcd_connect_status, GPIO_LCD_CONNECT);
    first_flg = FALSE;
  }
  return(oem_lcd_connect_status);
#else
  return(TRUE);
#endif
}

#define CHECK_DATA1 0x66

boolean dev_lcd_testmode_frex_check( void )
{
  static int frex_check_ng = FALSE;
  int     rc;
  uint32  read_data = 0;
  boolean ret = TRUE;

  if(frex_check_ng){
    return(FALSE);
  }

  printk(KERN_ERR "%s:start \n", __func__);

  rc = LCD_CtrlRead_Panel_S( 0x0C, read_data);
  if(rc < 0)
  {
    frex_check_ng = TRUE;
    ret = FALSE;
  }
  else
  {
    if(read_data != CHECK_DATA1)
    {
      printk(KERN_ERR "NG!!! read[%x]==check[%x] \n", read_data, CHECK_DATA1);
      ret = FALSE;
    }
  }
  printk(KERN_ERR "frex_check_drv %d:data[%x]end \n", ret, read_data);
  return ret ;
}

boolean oem_dev_lcd_testmode_frex_check( void )
{
	return dev_lcd_frex_check;
}

void debug_panel_read_DDB_start( void )
{
  return;
}

int oem_panel_read_power_mode( void )
{
  int     rc;
  uint32 read_data;
  rc = LCD_CtrlRead_Panel_S(0x0A, read_data);
  printk("%s():rc[%x]data 1[%x]\n", __func__, rc, read_data);
  if(rc < 0){
    return(rc);
  }
  return(read_data & 0xFF);
}


boolean oem_dev_lcd_status_get( void )
{
  return((boolean)g_device_lcd_state);
}

void debug_panel_read_gamma( void )
{
  return;
}
