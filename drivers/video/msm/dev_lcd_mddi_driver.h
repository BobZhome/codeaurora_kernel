/* This software is contributed or developed by KYOCERA Corporation. */
/* (C) 2012 KYOCERA Corporation                                      */

#ifndef DEV_LCD_MDDI_DRIVER_H
#define DEV_LCD_MDDI_DRIVER_H

#include "msm_fb_def.h"
#define GPIO_LCD_BACKLIGHT_EN    33 
#define GPIO_LCD_VSYNC           19
#define GPIO_LCD_RESET           18

#define CONFIG_LCD_CONNECT_CHECK

extern boolean oem_dev_lcd_testmode_frex_check( void );
extern boolean oem_dev_lcd_status_get( void );
extern void    oem_dev_lcd_backlight_enable(void);
extern void    oem_dev_lcd_backlight_disable(void);
extern boolean oem_dev_lcd_connect_check( void );
extern void    oem_dev_lcd_power_off( void );
extern void    oem_dev_lcd_start_address_set( void );
extern void    oem_dev_lcdbl_set(int lvl);
extern void    oem_dev_lcdbl_first_blon( void );

extern void oem_dev_lcd_wakelock( int type );
extern void oem_dev_lcd_wakeunlock( int type );

extern int  oem_lcd_update_flg;
extern void oem_lcd_busclk_fix( void );

extern boolean lcd_is_shutdown; 

#endif /* DEV_MDDI_DRIVER_H */
