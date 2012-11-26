/* drivers/rtc/alarm-dev.c
 *
 * Copyright (C) 2007-2009 Google, Inc.
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
 * (C) 2012 KYOCERA Corporation
*/

#include <asm/mach/time.h>
#include <linux/android_alarm.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/sysdev.h>
#include <linux/uaccess.h>
#include <linux/wakelock.h>


extern void alarm_clear_pending(void);

static long usertime_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int rv = 0;
        int utime_flg = -1;

	switch (cmd) {
        case ANDROID_ALARM_SET_USRTIME_AUTO:
		if (copy_from_user(&utime_flg, (void __user *)arg, sizeof(utime_flg))) {
			rv = -EFAULT;
			goto err1;
		}
                set_usertime_setting(utime_flg);
                alarm_clear_pending();
                printk("%s SET_USRTIME_AUTO[1:auto 0:manual] %d utimeflg:%d \n",__func__,get_usertime_setting(),utime_flg);
                break;
	default:
		rv = -EINVAL;
		goto err1;
	}
err1:
	return rv;
}

static int usertime_open(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static int usertime_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations usertime_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = usertime_ioctl,
	.open = usertime_open,
	.release = usertime_release,
};

static struct miscdevice usertime_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "usertime",
	.fops = &usertime_fops,
};

static int __init usertime_dev_init(void)
{
	int err;

	err = misc_register(&usertime_device);
	if (err)
		return err;

	return 0;
}

static void  __exit usertime_dev_exit(void)
{
	misc_deregister(&usertime_device);
}

module_init(usertime_dev_init);
module_exit(usertime_dev_exit);


