/* include/asm/mach-msm/htc_pwrsink.h
 *
 * Copyright (C) 2008 HTC Corporation.
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2011 Code Aurora Forum. All rights reserved.
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C)2012 KYOCERA Corporation
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/sched.h>
#include "pmic.h"
#include "timed_output.h"
#include <linux/wakelock.h>
#if defined(CONFIG_SPKAMP_CORE)
#include <linux/mfd/spkamp.h>
#endif /* defined(CONFIG_SPKAMP_CORE) */
#include <mach/msm_rpcrouter.h>

#if defined(CONFIG_TARGET_PROTOTYPE_WS2)
#include <mach/mpp.h>
#include <mach/hs_io_ctl_a.h>
#endif

#define PM_LIBPROG      0x30000061
#if (CONFIG_MSM_AMSS_VERSION == 6220) || (CONFIG_MSM_AMSS_VERSION == 6225)
#define PM_LIBVERS      0xfb837d0b
#else
#define PM_LIBVERS      0x10001
#endif

#define HTC_PROCEDURE_SET_VIB_ON_OFF	21
#define PMIC_VIBRATOR_LEVEL   (1300)

#define PMIC_VIBRATOR_LEVEL_HAPTICS (2000)

#ifdef CONFIG_KYOCERA_ORIGINAL_FEATURE
static struct delayed_work work_vibrator_off;
#else
static struct work_struct work_vibrator_on;
static struct work_struct work_vibrator_off;
#endif /* CONFIG_KYOCERA_ORIGINAL_FEATURE */ 
static struct hrtimer vibe_timer;
static struct wake_lock vib_wake_lock;

static void pm_mpp_setting_for_vib(void);
extern int vib_mpp_config(void);

#define VIB_STATUS_ON  (1)
#define VIB_STATUS_OFF (0)

static atomic_t vib_sts;
static atomic_t vib_level;
#define VIB_HAPTICS_ON  (1)
#define VIB_HAPTICS_OFF (0)
#define HAPTICS_MSEC (50)

#ifdef CONFIG_PM8XXX_RPC_VIBRATOR
static void set_pmic_vibrator(int on)
{
	int rc;

	rc = pmic_vib_mot_set_mode(PM_VIB_MOT_MODE__MANUAL);
	if (rc) {
		pr_err("%s: Vibrator set mode failed", __func__);
		return;
	}

	if (on)
		rc = pmic_vib_mot_set_volt(PMIC_VIBRATOR_LEVEL);
	else
		rc = pmic_vib_mot_set_volt(0);

	if (rc)
		pr_err("%s: Vibrator set voltage level failed", __func__);
}
#else
#ifdef CONFIG_KYOCERA_ORIGINAL_FEATURE
static void set_pmic_vibrator(int on)
{
	if (on)
	{
		HS_A_VIB_EN_ON();
		atomic_set(&vib_sts, VIB_STATUS_ON);
	}
	else
	{
		HS_A_VIB_EN_OFF();
		if(atomic_read(&vib_level))
		{
			pmic_vib_mot_set_volt(PMIC_VIBRATOR_LEVEL);
			atomic_set(&vib_level,VIB_HAPTICS_OFF);
		}
		atomic_set(&vib_sts, VIB_STATUS_OFF);
	}
}
#else /* CONFIG_KYOCERA_ORIGINAL_FEATURE */
static void set_pmic_vibrator(int on)
{
	static struct msm_rpc_endpoint *vib_endpoint;
	struct set_vib_on_off_req {
		struct rpc_request_hdr hdr;
		uint32_t data;
	} req;

	if (!vib_endpoint) {
		vib_endpoint = msm_rpc_connect(PM_LIBPROG, PM_LIBVERS, 0);
		if (IS_ERR(vib_endpoint)) {
			printk(KERN_ERR "init vib rpc failed!\n");
			vib_endpoint = 0;
			return;
		}
	}


	if (on)
		req.data = cpu_to_be32(PMIC_VIBRATOR_LEVEL);
	else
		req.data = cpu_to_be32(0);

	msm_rpc_call(vib_endpoint, HTC_PROCEDURE_SET_VIB_ON_OFF, &req,
		sizeof(req), 5 * HZ);
}
#endif
#endif
#ifdef CONFIG_KYOCERA_ORIGINAL_FEATURE
#else
static void pmic_vibrator_on(struct work_struct *work)
{
	set_pmic_vibrator(1);
}
#endif /* CONFIG_KYOCERA_ORIGINAL_FEATURE */

static void pmic_vibrator_off(struct work_struct *work)
{
	set_pmic_vibrator(0);
#ifdef CONFIG_KYOCERA_ORIGINAL_FEATURE
	wake_unlock(&vib_wake_lock);
#endif /* CONFIG_KYOCERA_ORIGINAL_FEATURE */
}
#ifdef CONFIG_KYOCERA_ORIGINAL_FEATURE
#else
static void timed_vibrator_on(struct timed_output_dev *sdev)
{
	wake_lock(&vib_wake_lock);
	schedule_work(&work_vibrator_on);
}

static void timed_vibrator_off(struct timed_output_dev *sdev)
{
	schedule_work(&work_vibrator_off);
	wake_unlock(&vib_wake_lock);
}
#endif /* CONFIG_KYOCERA_ORIGINAL_FEATURE */

#ifdef CONFIG_KYOCERA_ORIGINAL_FEATURE
static void try_vibrator_on(int value)
{
	if(VIB_STATUS_OFF == atomic_read(&vib_sts))
	{
		wake_lock(&vib_wake_lock);

		if(value <= HAPTICS_MSEC)
		{
			if(!atomic_read(&vib_level))
			{
				pmic_vib_mot_set_volt(PMIC_VIBRATOR_LEVEL_HAPTICS);
				atomic_set(&vib_level,VIB_HAPTICS_ON);
			}
		}
		set_pmic_vibrator(1);
	}
	else
	{
		if(value > HAPTICS_MSEC)
		{
			if(atomic_read(&vib_level))
			{
				set_pmic_vibrator(0);
				set_pmic_vibrator(1);
			}
		}
	}
	cancel_delayed_work_sync(&work_vibrator_off);
	schedule_delayed_work(&work_vibrator_off, msecs_to_jiffies(value));
}
static void try_vibrator_off(void)
{
	if(VIB_STATUS_ON == atomic_read(&vib_sts))
	{
		if(atomic_read(&vib_level))
		{
			pmic_vib_mot_set_volt(PMIC_VIBRATOR_LEVEL);
			atomic_set(&vib_level,VIB_HAPTICS_OFF);
		}
		cancel_delayed_work_sync(&work_vibrator_off);
		schedule_delayed_work(&work_vibrator_off, msecs_to_jiffies(1));
	}
}
#endif /* CONFIG_KYOCERA_ORIGINAL_FEATURE */
#ifdef CONFIG_KYOCERA_ORIGINAL_FEATURE
static void vibrator_enable(struct timed_output_dev *dev, int value)
{
	hrtimer_cancel(&vibe_timer);
	if(value != 0)
	{
		value = (value > 15000 ? 15000 : value);
		try_vibrator_on(value);
		hrtimer_start(&vibe_timer,
			      ktime_set(value / 1000, (value % 1000) * 1000000),
			      HRTIMER_MODE_REL);
	}
	else
	{
		try_vibrator_off();
	}
}
#else
static void vibrator_enable(struct timed_output_dev *dev, int value)
{
	hrtimer_cancel(&vibe_timer);

	if (value == 0)
		timed_vibrator_off(dev);
	else {
		value = (value > 15000 ? 15000 : value);

		timed_vibrator_on(dev);
		wake_lock(&vib_wake_lock);

		hrtimer_start(&vibe_timer,
			      ktime_set(value / 1000, (value % 1000) * 1000000),
			      HRTIMER_MODE_REL);
	}
}
#endif /* CONFIG_KYOCERA_ORIGINAL_FEATURE */
static int vibrator_get_time(struct timed_output_dev *dev)
{
	if (hrtimer_active(&vibe_timer)) {
		ktime_t r = hrtimer_get_remaining(&vibe_timer);
		struct timeval t = ktime_to_timeval(r);
		return t.tv_sec * 1000 + t.tv_usec / 1000;
	}
	return 0;
}

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
#ifdef CONFIG_KYOCERA_ORIGINAL_FEATURE
#else
	timed_vibrator_off(NULL);
#endif /* CONFIG_KYOCERA_ORIGINAL_FEATURE */
	return HRTIMER_NORESTART;
}

static struct timed_output_dev pmic_vibrator = {
	.name = "vibrator",
	.get_time = vibrator_get_time,
	.enable = vibrator_enable,
};

static int __init msm_init_pmic_vibrator(void)
{
#ifdef CONFIG_KYOCERA_ORIGINAL_FEATURE
	atomic_set(&vib_sts, VIB_STATUS_OFF);
	atomic_set(&vib_level,VIB_HAPTICS_OFF);
	INIT_DELAYED_WORK(&work_vibrator_off, pmic_vibrator_off);
#else
	INIT_WORK(&work_vibrator_on, pmic_vibrator_on);
	INIT_WORK(&work_vibrator_off, pmic_vibrator_off);
#endif
	wake_lock_init(&vib_wake_lock, WAKE_LOCK_SUSPEND, "vibrator");

	hrtimer_init(&vibe_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vibe_timer.function = vibrator_timer_func;
	timed_output_dev_register(&pmic_vibrator);
	pm_mpp_setting_for_vib();
	return 0;
}
static void pm_mpp_setting_for_vib(void)
{
	pmic_vib_mot_set_volt(0);
	vib_mpp_config();
	pmic_vib_mot_set_mode(PM_VIB_MOT_MODE__DBUS2);
	pmic_vib_mot_set_polarity(PM_VIB_MOT_POL__ACTIVE_HIGH);
	pmic_vib_mot_set_volt(PMIC_VIBRATOR_LEVEL);
}

device_initcall(msm_init_pmic_vibrator);
MODULE_DESCRIPTION("timed output pmic vibrator device");
MODULE_LICENSE("GPL");

