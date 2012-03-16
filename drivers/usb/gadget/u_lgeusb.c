/* linux/drivers/usb/gadget/u_lgeusb.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Copyright (C) 2011 LGE.
 * Author : Hyeon H. Park <hyunhui.park@lge.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <mach/msm_hsusb.h>
#include <mach/rpc_hsusb.h>
#ifdef CONFIG_USB_ANDROID
#include <linux/usb/android_composite.h>
#endif
#include <mach/board.h>
#ifdef CONFIG_MACH_LGE
#include <mach/board_lge.h>
#endif

/* #define LGEUSB_DEBUG */
/* #define LGEUSB_DYNAMIC_DEBUG */

#include "u_lgeusb.h"
/* LGE_CHANGE
 * To check factory mode in user space.
 * 2011-02-10, hyunhui.park@lge.com
 */
static struct mutex lock;

static int lgeusb_get_mode(char *buffer, struct kernel_param *kp);

/* LGE_CHANGES_S [khlee@lge.com] 2010-01-04, [VS740] usb switch */
/* to supports FS USB in the Factory ( LT cable will be connected) */
#define LG_FACTORY_CABLE_TYPE 		3
#define LG_FACTORY_CABLE_130K_TYPE 	10
#define LT_ADB_CABLE 			0xff

/* Read only */
module_param_call(mode, NULL, lgeusb_get_mode, NULL, S_IRUGO);
MODULE_PARM_DESC(mode, "LGE USB Specific mode");

static struct lgeusb_info *usb_info;
/* FIXME: This length must be same as MAX_STR_LEN in android.c */
#define MAX_SERIAL_NO_LEN 20

/* -------- CDMA Class Utils -------- */
#define LGE_PIF_CABLE 2

#ifdef CONFIG_USB_SUPPORT_LGE_GADGET_CDMA

extern int msm_chg_LG_cable_type(void);
extern void msm_get_MEID_type(char* sMeid);

static int get_serial_number(char *serial_number)
{
/* LGE_CHANGES_S [younsuk.song@lge.com] 2010-06-21, Set MEID */
	memset(serial_number, 0, MAX_SERIAL_NO_LEN);

	msm_get_MEID_type(serial_number);

	if(!strcmp(serial_number,"00000000000000")) 
		serial_number[0] = '\0';

	if(msm_chg_LG_cable_type() == LT_ADB_CABLE)
	{
		sprintf(serial_number,"%s","LGE_ANDROID_DE");
	}

  /* if LT cable, set serial_number to NULL */
	if (lgeusb_detect_factory_cable()) {
		serial_number[0] = '\0';
	}
	return 0;
/* LGE_CHANGES_E [younsuk.song@lge.com] 2010-06-21 */
}

/*
 * CDMA class model must detect 2 types of factory cable
 * 1. LT CABLE - must be FullSpeed setting
 * 2. 130K CABLE - must be HighSpeed setting
 *
 * In case of normal USB cable, we return 0.
 */
static int get_factory_cable(void)
{
/*LGSI_CHANGE_S <pranav.s@lge.com> PIF cable detection change*/
 int cable_type =  msm_chg_LG_cable_type();

	/* LGE_CHANGES_S [moses.son@lge.com] 2011-02-09, need to check LT cable type */
	if( cable_type == LG_FACTORY_CABLE_TYPE ||
	    cable_type == LG_FACTORY_CABLE_130K_TYPE)  //detect LT cable
		return 1;
	else
		return 0;
/*LGSI_CHANGE_E <pranav.s@lge.com> PIF cable detection change*/
}

#endif /* CONFIG_USB_SUPPORT_LGE_GADGET_CDMA */

#ifdef CONFIG_USB_SUPPORT_LGE_GADGET_GSM
static int get_serial_number(char *serial_number)
{
	unsigned char nv_imei_ptr[MAX_IMEI_LEN];
	int ret = -1;

	ret = msm_nv_imei_get(nv_imei_ptr);
	if (ret < 0) {
		nv_imei_ptr[0] = '\0';
		lgeusb_info("IMEI is NULL\n");
	} else {
		lgeusb_info("IMEI %s\n", nv_imei_ptr);
	}

	if (nv_imei_ptr[0] != '\0') {
		if ((nv_imei_ptr[0] == '8') && (nv_imei_ptr[1] == '0') &&
				(nv_imei_ptr[2] == 'A')) {
			memset(serial_number, 0, MAX_SERIAL_NO_LEN);
			/* We set serialno include header "80A" */
			memcpy(serial_number, nv_imei_ptr, MAX_IMEI_LEN);
			return 0;
		} else {
			serial_number[0] = '\0';
		}
	} else {
		serial_number[0] = '\0';
	}

	return ret;
}

static int get_factory_cable(void)
{
	int pif_detect = 0;

#ifdef CONFIG_LGE_DETECT_PIF_PATCH
	pif_detect = lge_get_pif_info();
#endif
	lgeusb_info("Using PIF ZIG (%d)\n", pif_detect);

	if (pif_detect == LGE_PIF_CABLE)
		return LGE_FACTORY_CABLE_TYPE;
	else
		return 0;
}
#endif /* CONFIG_USB_SUPPORT_LGE_GADGET_GSM */

static int lgeusb_get_mode(char *buffer, struct kernel_param *kp)
{
	int ret;
	struct lgeusb_info *info = usb_info;

	mutex_lock(&lock);
	ret = sprintf(buffer, "%s",
			(info->current_mode == LGEUSB_FACTORY_MODE
			 ? "factory" : "normal"));
	mutex_unlock(&lock);

	return ret;
}

static void do_switch_mode(int pid, int need_reset)
{
	struct lgeusb_info *info = usb_info;

	lgeusb_info("do_switch_mode : pid %x, need_reset %d\n", pid, need_reset);
	info->switch_func(pid, need_reset);
}

/* LGE_CHANGE
 * If factory cable (PIF or LT) is connected,
 * return 1, otherwise return 0.
 * 2011-01-13, hyunhui.park@lge.com
 */
int lgeusb_detect_factory_cable(void)
{
	return get_factory_cable();
}

/* LGE_CHANGE
 * If factory dedicated cable is connected,
 * switch to LGE usb factory mode.
 * 2011-01-13, hyunhui.park@lge.com
 */
void lgeusb_switch_factory_mode(int need_reset)
{
	struct lgeusb_info *info = usb_info;

	info->current_mode = LGEUSB_FACTORY_MODE;
	info->current_pid = info->get_pid();

	do_switch_mode(LGE_FACTORY_PID, need_reset);
}

/* LGE_CHANGE
 * If a normal cable is connected,
 * switch to android mode back.
 * 2011-01-13, hyunhui.park@lge.com
 */
void lgeusb_switch_android_mode(int need_reset)
{
	struct lgeusb_info *info = usb_info;
	int restore_pid = info->current_pid;

	info->current_mode = LGEUSB_ANDROID_MODE;
	do_switch_mode(restore_pid, need_reset);
}

/* LGE_CHANGE
 * Get current mode(factory or android).
 * 2011-01-24, hyunhui.park@lge.com
 */
int lgeusb_get_current_mode(void)
{
	struct lgeusb_info *info = usb_info;

	return info->current_mode;
}

/* LGE_CHANGE
 * 1. If cable is factory cable, switch manufacturing mode.
 * 2. Get serial number from CP and set product id to CP.
 * 2011-01-13, hyunhui.park@lge.com
 */
int lgeusb_set_current_mode(int need_reset)
{
	struct lgeusb_info *info = usb_info;
	int ret;

	if (!info->serialno || !info->defaultno) {
		lgeusb_info("serial numbers are invalid, skip configuration.\n");
		return -EINVAL;
	}

	if (get_factory_cable()) {
		/* We already are in factory mode, skip it. */
		if (info->current_mode == LGEUSB_FACTORY_MODE)
			return LGE_FACTORY_PID;

		/* When manufacturing, do not use serial number */
		lgeusb_info("We detect LGE factory cable......\n");
		lgeusb_switch_factory_mode(need_reset);
		msm_hsusb_send_productID(LGE_FACTORY_PID);
		msm_hsusb_is_serial_num_null(1);
		info->serialno[0] = '\0';
		return LGE_FACTORY_PID;
	}

	/* We already are in android mode, skip it. */
	if (info->current_mode == LGEUSB_ANDROID_MODE)
		return info->current_pid;

	lgeusb_info("We detect Normal USB cable......\n");
	lgeusb_switch_android_mode(need_reset);

	ret = get_serial_number(info->serialno);

	msm_hsusb_send_productID(info->current_pid);
	msm_hsusb_is_serial_num_null(0);

	if (!ret && (info->serialno[0] != '\0'))
		msm_hsusb_send_serial_number(info->serialno);
	else
		msm_hsusb_send_serial_number(info->defaultno);

	if (ret < 0)
		lgeusb_info("fail to get serial number, set to default.\n");

	return info->current_pid;
}

/* LGE_CHANGE
 * Register lge usb information(which include callback functions).
 * 2011-01-14, hyunhui.park@lge.com
 */
void lgeusb_register_usbinfo(struct lgeusb_info *info)
{
	if (info) {
		usb_info = info;
		lgeusb_info("Registering infomation for lgeusb is success\n");

		lgeusb_debug("switch_func %p, get_pid %p\n",
				usb_info->switch_func,
				usb_info->get_pid);
	} else {
		lgeusb_info("Registering infomation for lgwusb is failed\n");
	}
}

static int __init lgeusb_init(void)
{
	lgeusb_info("u_lgeusb init\n");
	mutex_init(&lock);

	return 0;
}
module_init(lgeusb_init);
