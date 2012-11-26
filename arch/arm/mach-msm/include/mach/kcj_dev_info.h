/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
*/
#ifndef _KCJ_DEV_INFO_H_
#define _KCJ_DEV_INFO_H_

typedef enum {
	DEV_INFO_CAM_OFF,
	DEV_INFO_CAM_PREVIEW,
	DEV_INFO_CAM_STATE_MAX
} dev_info_cam_state_type;

typedef enum {
	DEV_INFO_MOVIE_OFF,
	DEV_INFO_MOVIE_REC_HIGH,
	DEV_INFO_MOVIE_REC_MEDIUM_WIDE,
	DEV_INFO_MOVIE_REC_MEDIUM,
	DEV_INFO_MOVIE_REC_LOW,
	DEV_INFO_MOVIE_PLAY,
	DEV_INFO_MOVIE_STATE_MAX
} dev_info_movie_state_type;

typedef enum {
	DEV_INFO_WLAN_OFF,
	DEV_INFO_WLAN_ON,
	DEV_INFO_WLAN_HOTSPOT_ON,
	DEV_INFO_WLAN_STATE_MAX
} dev_info_wlan_state_type;

typedef enum {
	DEV_INFO_SND_OFF,
	DEV_INFO_SND_SPEAKER,
	DEV_INFO_SND_RECEIVER,
	DEV_INFO_SND_HEADSET,
	DEV_INFO_SND_STATE_MAX
} dev_info_snd_state_type;


/********************/
/* ioctl definition */
/********************/
#define DEV_INFO_IOCTL_MAGIC	0xF7

#define DEV_INFO_PATH	"/dev/kcj_dev_info"

#define DEV_INFO_MOVIE		_IOW(DEV_INFO_IOCTL_MAGIC, 1, dev_info_movie_state_type )

#define DEV_INFO_SND_VOL	_IOW(DEV_INFO_IOCTL_MAGIC, 2, unsigned int )


extern void kcj_dev_info_update_lcd_lv( unsigned int lv, int updown );

extern void kcj_dev_info_update_lcd_onoff( bool onoff );

extern void kcj_dev_info_update_cam( dev_info_cam_state_type cam_st );

extern void kcj_dev_info_update_wlan( dev_info_wlan_state_type wlan_st );

extern void kcj_dev_info_update_snd( dev_info_snd_state_type snd_st, bool onoff );

extern void kcj_dev_info_update_snd_vol( unsigned int vol );

extern void kcj_dev_info_update_touch_panel( bool push );

extern void kcj_dev_info_update_flash_led( bool onoff );

#endif /* _KCJ_DEV_INFO_H_ */
