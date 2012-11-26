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
#include "api_nv.h"
#include "proc_comm.h"

#define NV_STATUS_DEBUG(...) printk(__VA_ARGS__)

static int read_nv_d_access_info(smem_nv_d_access_info_type *nv_d_access_info);
static int write_nv_d_access_info(void);

struct nv_d_device_t {
	struct miscdevice misc;
};

spinlock_t smem_nv_d_access_lock;

static long nv_d_ioctl(struct file *file,
				unsigned int cmd, unsigned long arg)
{
	struct nv_d_ioctl_info info;
    unsigned long result = -EFAULT;

	switch(cmd)
	{
		case IOCTL_NV_D_INFO_READ_CMD:
			result = copy_from_user(&info,(struct nv_d_ioctl_info __user *)arg,sizeof(info));
			NV_STATUS_DEBUG("@@@IOCTL_NV_D_INFO_READ_CMD\n");
			if( 0 != read_nv_d_access_info(&info.nv_d_access_info))
			{
				NV_STATUS_DEBUG("@@@DEBUG NV_PRODUCT_LINE_I get error !!!\n");
				return -EINVAL;
			}

			result = copy_to_user((struct nv_d_ioctl_info __user *)arg,&info,sizeof(info));
			break;

		case IOCTL_NV_D_INFO_WRITE_CMD:
			NV_STATUS_DEBUG("@@@IOCTL_NV_D_INFO_WRITE_CMD\n");
			if( 0 != write_nv_d_access_info())
			{
				NV_STATUS_DEBUG("@@@DEBUG UPDATE_FLAG write error !!!\n");
				return -EINVAL;
			}

			break;
		default:
		    return -EINVAL;
			break;
	}
	return 0;
}

static int read_nv_d_access_info(smem_nv_d_access_info_type *nv_d_access_info)
{
    smem_nv_d_access_info_type *info_ptr;
	unsigned long flags;

	memset(nv_d_access_info, 0x00, sizeof(smem_nv_d_access_info_type));

	info_ptr = smem_alloc(SMEM_NV_D_ACCESS_INFO,
						 sizeof(smem_nv_d_access_info_type));
	if(!info_ptr) {
		printk(KERN_ERR "(%s): smem_alloc() ERR !!\n", __func__);
		return -EINVAL;
	}
	spin_lock_irqsave(&smem_nv_d_access_lock, flags);
	memcpy(nv_d_access_info, info_ptr, sizeof(smem_nv_d_access_info_type));
	spin_unlock_irqrestore(&smem_nv_d_access_lock, flags);

	return 0;
}

static int write_nv_d_access_info(void)
{
    smem_nv_d_access_info_type *info_ptr;
	unsigned long flags;

	info_ptr = smem_alloc(SMEM_NV_D_ACCESS_INFO,
						 sizeof(smem_nv_d_access_info_type));
	if(!info_ptr) {
		printk(KERN_ERR "(%s): smem_alloc() ERR !!\n", __func__);
		return -EINVAL;
	}
	spin_lock_irqsave(&smem_nv_d_access_lock, flags);
	info_ptr->product_line_d_info.update_flag = NV_D_ACCESS_UPDATE_OFF;
	info_ptr->boot_debug_d_info.update_flag   = NV_D_ACCESS_UPDATE_OFF;
	spin_unlock_irqrestore(&smem_nv_d_access_lock, flags);

	return 0;
}

static int nv_d_open(struct inode *ip, struct file *fp)
{
	return nonseekable_open(ip, fp);
}

static int nv_d_release(struct inode *ip, struct file *fp)
{
	return 0;
}

static const struct file_operations nv_d_fops = {
	.owner = THIS_MODULE,
	.open = nv_d_open,
	.unlocked_ioctl = nv_d_ioctl,
	.release = nv_d_release,
};

static struct nv_d_device_t nv_d_device = {
	.misc = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "nv_d_info",
		.fops = &nv_d_fops,
	}
};

static void __exit nv_d_exit(void)
{
	misc_deregister(&nv_d_device.misc);
}

static int __init nv_d_init(void)
{
	int ret;
	ret = misc_register(&nv_d_device.misc);
	return ret;
}

module_init(nv_d_init);
module_exit(nv_d_exit);

MODULE_DESCRIPTION("NV ITEM READ Driver for Direct NV access");
MODULE_LICENSE("GPL v2");

