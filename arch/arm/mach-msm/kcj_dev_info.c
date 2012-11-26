/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
*/

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/miscdevice.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>

#include <mach/msm_smd.h>
#include <mach/kcj_dev_info.h>
#include <mach/proc_comm_kyocera.h>


#define CHECK_DIF_LV	9


struct dev_info_device_t {
	struct miscdevice misc;
};

static u32 dev_info_lcd		= 0x000100FF;
static u32 dev_info_cam		= 0;
static u32 dev_info_movie	= 0;
static u32 dev_info_wlan	= 0;
static u32 dev_info_snd		= 0;
static u32 dev_info_snd_vol	= 0;
static u32 dev_info_touch	= 0;
static u32 dev_info_flash	= 0;


/****************************************************************************
 *  FUNCTION : void update_dev_info_lcd                                      
 ****************************************************************************/
static void update_dev_info_lcd( void )
{
	proc_comm_rpc_apps_to_modem( PROC_COMM_SUB_CMD_UPDATE_DEV_INFO_LCD,
	&dev_info_lcd );
}

/****************************************************************************
 *  FUNCTION : void update_dev_info_cam                                      
 ****************************************************************************/
static void update_dev_info_cam( void )
{
	proc_comm_rpc_apps_to_modem( PROC_COMM_SUB_CMD_UPDATE_DEV_INFO_CAM,
	&dev_info_cam );
}

/****************************************************************************
 *  FUNCTION : void update_dev_info_movie                                    
 ****************************************************************************/
static void update_dev_info_movie( void )
{
	proc_comm_rpc_apps_to_modem( PROC_COMM_SUB_CMD_UPDATE_DEV_INFO_MOVIE,
	&dev_info_movie );
}

/****************************************************************************
 *  FUNCTION : void update_dev_info_wlan                                     
 ****************************************************************************/
static void update_dev_info_wlan( void )
{
	proc_comm_rpc_apps_to_modem( PROC_COMM_SUB_CMD_UPDATE_DEV_INFO_WLAN,
	&dev_info_wlan );
}

/****************************************************************************
 *  FUNCTION : void update_dev_info_snd                                      
 ****************************************************************************/
static void update_dev_info_snd( void )
{
	proc_comm_rpc_apps_to_modem(PROC_COMM_SUB_CMD_UPDATE_DEV_INFO_SND,
	&dev_info_snd );
}

/****************************************************************************
 *  FUNCTION : void update_dev_info_snd_vol                                  
 ****************************************************************************/
static void update_dev_info_snd_vol( void )
{
	proc_comm_rpc_apps_to_modem(PROC_COMM_SUB_CMD_UPDATE_DEV_INFO_SND_VOL,
	&dev_info_snd_vol );
}

/****************************************************************************
 *  FUNCTION : void update_dev_info_touch_panel                              
 ****************************************************************************/
static void update_dev_info_touch_panel( void )
{
	proc_comm_rpc_apps_to_modem(PROC_COMM_SUB_CMD_UPDATE_DEV_INFO_TOUCH,
	&dev_info_touch );
}

/****************************************************************************
 *  FUNCTION : void update_dev_info_flash_led                                
 ****************************************************************************/
static void update_dev_info_flash_led( void )
{
	proc_comm_rpc_apps_to_modem(PROC_COMM_SUB_CMD_UPDATE_DEV_INFO_FLASH,
	&dev_info_flash );
}

/****************************************************************************
 *  FUNCTION : void kcj_dev_info_update_snd                                  
 ****************************************************************************/
void kcj_dev_info_update_snd( dev_info_snd_state_type snd_st, bool onoff )
{
	switch( snd_st )
	{
		case DEV_INFO_SND_SPEAKER:
		case DEV_INFO_SND_RECEIVER:
		case DEV_INFO_SND_HEADSET:
			if( onoff )
			{
				dev_info_snd |= ( 1UL << snd_st );
			}
			else
			{
				dev_info_snd &= ~( 1UL << snd_st );
			}
			break;

		case DEV_INFO_SND_OFF:
		default:
			dev_info_snd = 0;
			break;
	}
	update_dev_info_snd();
	printk( "%s() dev_info_snd:%d snd_st:%d onoff:%d\n", __func__, dev_info_snd, snd_st, onoff );
}
EXPORT_SYMBOL(kcj_dev_info_update_snd);

/****************************************************************************
 *  FUNCTION : void kcj_dev_info_update_snd_vol                              
 ****************************************************************************/
void kcj_dev_info_update_snd_vol( unsigned int vol )
{
	dev_info_snd_vol = vol;
	update_dev_info_snd_vol();
	printk( "%s() dev_info_snd_vol:%d\n", __func__, dev_info_snd_vol );
}
EXPORT_SYMBOL(kcj_dev_info_update_snd_vol);

/****************************************************************************
 *  FUNCTION : void kcj_dev_info_update_lcd_lv                               
 ****************************************************************************/
void kcj_dev_info_update_lcd_lv( unsigned int lv, int updown )
{
	static u32 send_dev_info_lcd_lv_up   = 0;
	static u32 send_dev_info_lcd_lv_down = 0;
	static bool lcd_open = false;
	bool update = false;

	if( updown == 1 )
	{
		dev_info_lcd &= 0xFFFFFF00;
		dev_info_lcd |= lv;

		if( lcd_open )
		{
			return;
		}

		if( send_dev_info_lcd_lv_up != lv )
		{
			if( send_dev_info_lcd_lv_up > lv )
			{
				if( ( send_dev_info_lcd_lv_up - lv ) > CHECK_DIF_LV )
				{
					update = true;
				}
			}
			else if( send_dev_info_lcd_lv_up < lv )
			{
				if( ( lv - send_dev_info_lcd_lv_up ) > CHECK_DIF_LV )
				{
					update = true;
				}
			}
		}
	}
	else
	{
		dev_info_lcd &= 0xFFFF00FF;
		dev_info_lcd |= lv << 8;

		if( send_dev_info_lcd_lv_down != lv )
		{
			if( send_dev_info_lcd_lv_down > lv )
			{
				if( ( send_dev_info_lcd_lv_down - lv ) > CHECK_DIF_LV )
				{
					update = true;
				}
			}
			else if( send_dev_info_lcd_lv_down < lv )
			{
				if( ( lv - send_dev_info_lcd_lv_down ) > CHECK_DIF_LV )
				{
					update = true;
				}
			}
		}

		if( lv != 0 )
		{
			lcd_open = true;
		}
		else
		{
			lcd_open = false;
		}
	}

	if( update )
	{
		update_dev_info_lcd();
		send_dev_info_lcd_lv_up   = ( dev_info_lcd & 0x000000FF );
		send_dev_info_lcd_lv_down = ( (dev_info_lcd & 0x0000FF00) >> 8 );
		printk( "%s() dev_info_lcd:%x lv:%d updown:%d lcd_open:%d\n",
		__func__, dev_info_lcd, lv, updown, lcd_open );
	}
}
EXPORT_SYMBOL(kcj_dev_info_update_lcd_lv);

/****************************************************************************
 *  FUNCTION : void kcj_dev_info_update_lcd_onoff                            
 ****************************************************************************/
void kcj_dev_info_update_lcd_onoff( bool onoff )
{
	dev_info_lcd &= 0xFF00FFFF;
	dev_info_lcd |= (u32)(onoff << 16);
	update_dev_info_lcd();
	printk( "%s() dev_info_lcd:%x lcd_onoff:%d\n", __func__, dev_info_lcd, onoff );
}
EXPORT_SYMBOL(kcj_dev_info_update_lcd_onoff);

/****************************************************************************
 *  FUNCTION : void kcj_dev_info_update_cam                                  
 ****************************************************************************/
void kcj_dev_info_update_cam( dev_info_cam_state_type cam_st )
{
	dev_info_cam = cam_st;
	update_dev_info_cam();
	printk( "%s() dev_info_cam:%d\n", __func__, dev_info_cam );
}
EXPORT_SYMBOL(kcj_dev_info_update_cam);

/****************************************************************************
 *  FUNCTION : void kcj_dev_info_update_wlan                                 
 ****************************************************************************/
void kcj_dev_info_update_wlan( dev_info_wlan_state_type wlan_st )
{
	dev_info_wlan = wlan_st;
	update_dev_info_wlan();
	printk( "%s() dev_info_wlan:%d\n", __func__, dev_info_wlan );
}
EXPORT_SYMBOL(kcj_dev_info_update_wlan);

/****************************************************************************
 *  FUNCTION : void kcj_dev_info_update_touch_panel                          
 ****************************************************************************/
void kcj_dev_info_update_touch_panel( bool push )
{
	dev_info_touch = (u32)push;
	update_dev_info_touch_panel();
	printk( "%s() dev_info_touch_panel:%d\n", __func__, dev_info_touch );
}
EXPORT_SYMBOL(kcj_dev_info_update_touch_panel);

/****************************************************************************
 *  FUNCTION : void kcj_dev_info_update_flash_led                            
 ****************************************************************************/
void kcj_dev_info_update_flash_led( bool onoff )
{
	dev_info_flash = (u32)onoff;
	update_dev_info_flash_led();
	printk( "%s() dev_info_flash:%d\n", __func__, dev_info_flash );
}
EXPORT_SYMBOL(kcj_dev_info_update_flash_led);

/****************************************************************************
 *  FUNCTION : void kcj_dev_info_update_movie                                
 ****************************************************************************/
static void kcj_dev_info_update_movie( dev_info_movie_state_type movie_st )
{
	dev_info_movie = movie_st;
	update_dev_info_movie();
	printk( "%s() dev_info_movie:%d\n", __func__, dev_info_movie );
}

/****************************************************************************
 *  FUNCTION : int dev_info_ioctl                                            
 ****************************************************************************/
static long dev_info_ioctl( struct file *file,
				unsigned int cmd, unsigned long arg )
{
	unsigned int data;

	switch( cmd )
	{
		case DEV_INFO_MOVIE:
			if( copy_from_user( &data, (void __user *)arg, sizeof(data) ) )
			{
				return -EFAULT;
			}
			kcj_dev_info_update_movie( data );
			printk( "DEV_INFO_MOVIE %s %d\n", __func__, data );
			break;

		case DEV_INFO_SND_VOL:
			if( copy_from_user( &data, (void __user *)arg, sizeof(data) ) )
			{
				return -EFAULT;
			}
			kcj_dev_info_update_snd_vol( data );
			printk( "DEV_INFO_SND_VOL %s %d\n", __func__, data );
			break;

		default:
			break;
	}
	return 0;
}

/****************************************************************************
 *  FUNCTION : int dev_info_open                                             
 ****************************************************************************/
static int dev_info_open( struct inode *ip, struct file *fp )
{
	return nonseekable_open( ip, fp );
}

/****************************************************************************
 *  FUNCTION : int dev_info_release                                          
 ****************************************************************************/
static int dev_info_release( struct inode *ip, struct file *fp )
{
	return 0;
}

static const struct file_operations dev_info_fops = {
	.owner		= THIS_MODULE,
	.open		= dev_info_open,
	.release	= dev_info_release,
	.unlocked_ioctl = dev_info_ioctl,
};

static struct dev_info_device_t dev_info_device = {
	.misc = {
		.minor	= MISC_DYNAMIC_MINOR,
		.name	= "kcj_dev_info",
		.fops	= &dev_info_fops,
	}
};

/****************************************************************************
 *  FUNCTION : int dev_info_init                                             
 ****************************************************************************/
static int __init dev_info_init(void)
{
	int ret;
	ret = misc_register( &dev_info_device.misc );
	return ret;
}

/****************************************************************************
 *  FUNCTION : void dev_info_exit                                            
 ****************************************************************************/
static void __exit dev_info_exit(void)
{
	misc_deregister( &dev_info_device.misc );
}

module_init(dev_info_init);
module_exit(dev_info_exit);

MODULE_DESCRIPTION("Device Information Driver");
MODULE_LICENSE("GPL v2");
