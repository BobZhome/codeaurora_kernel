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
#include <linux/mmc/host.h>
#include <mach/hs_io_ctl_a.h>
#include <mach/msm_smd.h>

#include <mach/smem_log.h>

#include "smd_private.h"
#include "api_sdcc_info.h"

//#define SMEM_FOTA_DEBUG(...)
#define SD_STATUS_DEBUG(...) printk(__VA_ARGS__)

extern int card_totalsize[MMC_HOST_MAX_INDEX];

extern unsigned int shared_manfid[MMC_HOST_MAX_INDEX];

struct sd_device_t {
	struct miscdevice misc;
};

static long sd_ioctl(struct file *file,
				unsigned int cmd, unsigned long arg)
{
	struct sd_status_ioctl_info info;
    unsigned long result = -EFAULT;

	switch(cmd)
	{
		case IOCTL_SD_STATUS_READ_CMD:
			result = copy_from_user(&info,(struct sd_status_ioctl_info __user *)arg,sizeof(info));
			SD_STATUS_DEBUG("@@@DEBUG IOCTL_SD_STATUS_READ_CMD\n");
			if( GPIO_LO == gpio_get_value(GPIO_CARD_DETB))
			{
				SD_STATUS_DEBUG("@@@DEBUG SDcard IN !!!\n");
				info.card_sts = SD_CARD_IN;
			}
			else
			{
				SD_STATUS_DEBUG("@@@DEBUG SDcard OUT !!!\n");
				info.card_sts = SD_CARD_OUT;
			}
			result = copy_to_user((struct sd_status_ioctl_info __user *)arg,&info,sizeof(info));
			break;

		case IOCTL_SD_SIZE_GET_CMD:
			result = copy_from_user(&info,(struct sd_status_ioctl_info __user *)arg,sizeof(info));
			SD_STATUS_DEBUG("@@@DEBUG IOCTL_SD_SIZE_GET_CMD\n");
			if( GPIO_LO != gpio_get_value(GPIO_CARD_DETB))
			{
				SD_STATUS_DEBUG("@@@DEBUG Size=0 (SDcard OUT) \n");
				info.card_total = 0;
			}
			else
			{
				info.card_total = card_totalsize[SD_INDEX];
				SD_STATUS_DEBUG("@@@DEBUG SDcard_size %d \n",card_totalsize[SD_INDEX]);
			}
			result = copy_to_user((struct sd_status_ioctl_info __user *)arg,&info,sizeof(info));
			break;

		case IOCTL_EMMC_NAND_DEVCODE_GET_CMD:
			result = copy_from_user(&info,(struct sd_status_ioctl_info __user *)arg,sizeof(info));
			SD_STATUS_DEBUG("@@@DEBUG IOCTL_MMC_DEVCODE_GET_CMD\n");

			info.manfid = shared_manfid[EMMC_INDEX];
			SD_STATUS_DEBUG("@@@DEBUG DeviceCode %d \n",shared_manfid[EMMC_INDEX]);

			result = copy_to_user((struct sd_status_ioctl_info __user *)arg,&info,sizeof(info));
			break;

		default:
			break;
	}
	return 0;
}

static int sd_open(struct inode *ip, struct file *fp)
{
	return nonseekable_open(ip, fp);
}

static int sd_release(struct inode *ip, struct file *fp)
{
	return 0;
}

static const struct file_operations sd_fops = {
	.owner = THIS_MODULE,
	.open = sd_open,
	.unlocked_ioctl = sd_ioctl,
	.release = sd_release,
};

static struct sd_device_t sd_device = {
	.misc = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "sd_status",
		.fops = &sd_fops,
	}
};

static void __exit sd_state_exit(void)
{
	misc_deregister(&sd_device.misc);
}

static int __init sd_state_init(void)
{
	int ret;
	ret = misc_register(&sd_device.misc);
	return ret;
}

module_init(sd_state_init);
module_exit(sd_state_exit);

MODULE_DESCRIPTION("SDcard_status Driver");
MODULE_LICENSE("GPL v2");
