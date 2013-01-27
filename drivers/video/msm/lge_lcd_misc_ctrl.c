/*  arch/arm/mach-msm/qdsp5v2/lge_audio_misc_ctl.c
 *
 * Copyright (C) 2009 LGE, Inc.
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
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/time.h>
/*LGE_CHANGE_S: kiran.kanneganti@lge.com 27-06-2012*/
/*Remove compile error when lcd tuning config enabled */
#include <linux/platform_device.h>
/*LGE_CHANGE_E: kiran.kanneganti@lge.com 27-06-2012*/
#include <mach/board_lge.h>
#include <mach/board.h>

#define IOCTL_READ_1ST _IOW('a', 0, int)
#define IOCTL_WRITE_1ST _IOW('a', 3, int)
/*LGE_CHANGE_S: kiran.kanneganti@lge.com 27-06-2012*/
/*Remove compile error when lcd tuning config enabled */
struct msm_panel_common_pdata *lcdc_pdata = NULL;
/*LGE_CHANGE_E: kiran.kanneganti@lge.com 27-06-2012*/
long device_ioctl(struct file *file, unsigned int ioctl_num,
		unsigned long ioctl_param)
{

	switch (ioctl_num) {

	case IOCTL_READ_1ST:
		printk(KERN_INFO "IOCTL_READ_1ST\n");
		lcdc_pdata->read_regset(ioctl_param);
		break;
	case IOCTL_WRITE_1ST:
		printk(KERN_INFO "IOCTL_WRITE_1ST\n");
		lcdc_pdata->write_regset(ioctl_param);
		break;
	}
	return 0;
}

static const struct file_operations lcd_misc_fops = {
	.owner	= THIS_MODULE,
	.unlocked_ioctl = device_ioctl
};

struct miscdevice lcd_misc_dev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "lcd_misc",
	.fops	= &lcd_misc_fops
};

static int lcd_misc_probe(struct platform_device *pdev)
{
	lcdc_pdata = pdev->dev.platform_data;
	return misc_register(&lcd_misc_dev);
}
static struct platform_driver this_driver = {
	.probe  = lcd_misc_probe,
	.driver = {
		.name   = "lcd_misc_msm",
	},
};

int __init lcd_misc_init(void)
{
	printk(KERN_INFO "lcd_misc_init \n");
	return platform_driver_register(&this_driver);
}

device_initcall(lcd_misc_init);

MODULE_DESCRIPTION("MSM MISC driver");
MODULE_LICENSE("GPL v2");
