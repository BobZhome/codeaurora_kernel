/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
*/

#include <asm/div64.h>
#include <linux/string.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/miscdevice.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <mach/hs_io_ctl_a.h>
#include <mach/smem_log.h>
#include "mmc_FOTA.h"


#define PRINT_PREF KERN_ERR "mmc_fota: "

static int dev;

module_param(dev, int, S_IRUGO);
MODULE_PARM_DESC(dev, "MTD device number to use");

#define BOARD_MMC_BLOCK_SIZE (64 * 1024)
#define FLASH_END_ADDR 0x80000000
#define MAX_DEV_NAME_LEN  22

//#define MMC_FOTA_DEBUG(...)
#define MMC_FOTA_DEBUG(...) printk(__VA_ARGS__)

static int  fota_flash_init ( void );
static int  fota_flash_read ( int openblk, loff_t r_addr, unsigned char *r_buff, size_t r_size );
static int  fota_flash_write ( int openblk, loff_t w_addr, unsigned char *w_buff, size_t w_size );

struct fota_device_t {
    struct miscdevice misc;
};

static long fota_ioctl(struct file *file,
				unsigned int cmd, unsigned long arg)
{
    unsigned long result = -EFAULT;
    struct mmc_fota_ioctl_info info;
    unsigned char *mid_buff;

    switch(cmd)
    {
	case IOCTL_MMC_FOTA_INIT_CMD:
	    result = copy_from_user(&info,(struct mmc_fota_ioctl_info __user *)arg,sizeof(info));
	    MMC_FOTA_DEBUG( "@@@DEBUG IOCTL_MMC_FOTA_INIT_CMD addr[%x] buf_addr[%x] size[%x]\n", (unsigned int)info.addr, (unsigned int)info.buff, info.size );

	    info.result = fota_flash_init();

	    MMC_FOTA_DEBUG("@@@DEBUG IOCTL_MMC_FOTA_INIT_CMD [%d]\n",info.result );
	    result = copy_to_user((struct mmc_fota_ioctl_info __user *)arg,&info,sizeof(info));
	    break;

	case IOCTL_MMC_FOTA_READ_CMD:
	    mid_buff = (char*)kmalloc(BOARD_MMC_BLOCK_SIZE, GFP_KERNEL);
	    if (!mid_buff)
	    {
			printk(KERN_ERR "fota_workbuf_read: Out of memory\n");
			info.size = 0;
	    }
	    else
	    {
			result = copy_from_user(&info,(struct mmc_fota_ioctl_info __user *)arg,sizeof(info));

			MMC_FOTA_DEBUG("@@@DEBUG IOCTL_MMC_FOTA_READ_CMD addr[%#llx] buf_addr[%x] size[%x]\n", (long long)info.addr, (unsigned int)info.buff, info.size );
			if(((info.addr & (BOARD_MMC_BLOCK_SIZE-1)) + info.size ) > BOARD_MMC_BLOCK_SIZE)
			{
		    	printk(KERN_ERR "Fota_Block_over_read\n");
		    	info.size = 0;
			}
			else
        	{
			    info.result = fota_flash_read ( info.openblk, info.addr, mid_buff, info.size);
			    if( info.result != 0 )
			    {
				    info.size = 0;
			    }
			    else
			    {
				    result = copy_to_user(info.buff, mid_buff, info.size);
		        }
			}
			kfree(mid_buff);
		}
	    MMC_FOTA_DEBUG("@@@DEBUG IOCTL_MMC_FOTA_READ_CMD [%d] [%d]\n",info.size, info.result);
	    result = copy_to_user((struct mmc_fota_ioctl_info __user *)arg,&info,sizeof(info));
	    break;

	case IOCTL_MMC_FOTA_WRITE_CMD:
	    mid_buff = (char*)kmalloc(BOARD_MMC_BLOCK_SIZE, GFP_KERNEL);
	    if (!mid_buff)
	    {
			printk(KERN_ERR "fota_workbuf_write: Out of memory\n");
			info.size = 0;
	    }
	    else
	    {
			result = copy_from_user(&info,(struct mmc_fota_ioctl_info __user *)arg,sizeof(info));

			MMC_FOTA_DEBUG("@@@DEBUG IOCTL_MMC_FOTA_WRITE_CMD addr[%#llx] buf_addr[%x] size[%x]\n", (long long)info.addr, (unsigned int)info.buff, info.size);
			if(((info.addr & (BOARD_MMC_BLOCK_SIZE-1)) + info.size ) > BOARD_MMC_BLOCK_SIZE)
			{
		    	printk(KERN_ERR "Fota_Block_over_write\n");
		   		info.size = 0;
			}
        	else
        	{
   	            result = copy_from_user(mid_buff, info.buff, info.size);

            	info.result = fota_flash_write ( info.openblk, info.addr, mid_buff, info.size);

   	            if(info.result != 0 )
   	            {
	            	info.size = 0;
    	        }
	        }
		    kfree(mid_buff);
	    }
	    MMC_FOTA_DEBUG("@@@DEBUG IOCTL_MMC_FOTA_WRITE_CMD [%x] [%d]\n",info.size, info.result );
	    result = copy_to_user((struct mmc_fota_ioctl_info __user *)arg,&info,sizeof(info));
	    break;

	default:
	    MMC_FOTA_DEBUG("@@@DEBUG default");
	    break;
    }
    return 0;
}

static int fota_open(struct inode *ip, struct file *fp)
{
    return nonseekable_open(ip, fp);
}

static int fota_release(struct inode *ip, struct file *fp)
{
    return 0;
}

static const struct file_operations fota_fops = {
    .owner = THIS_MODULE,
    .open = fota_open,
    .unlocked_ioctl = fota_ioctl,
    .release = fota_release,
};

static struct fota_device_t fota_device = {
    .misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "fota_fl",
	.fops = &fota_fops,
    }
};

static void __exit mmc_fota_exit(void)
{
    misc_deregister(&fota_device.misc);
}

static int __init mmc_fota_init(void)
{
    int ret;
    ret = misc_register(&fota_device.misc);
    return ret;
}

module_init(mmc_fota_init);
module_exit(mmc_fota_exit);

MODULE_DESCRIPTION("eMMC Memory FOTA Driver");
MODULE_LICENSE("GPL v2");

static int addr_check (loff_t addr)
{
	if(addr > FLASH_END_ADDR)
	{
		return -1;
	}
	return 0;
}


static int  fota_flash_init ( void )
{
    printk(KERN_INFO "\n");
    printk(KERN_INFO "=================================================\n");

    return 0;
}

static int  fota_flash_read ( int openblk, loff_t r_addr,
								 unsigned char *r_buff, size_t r_size )
{
    int ret = 0;
	struct file *fota_file;
	mm_segment_t  _segfs;
	loff_t seek_result = 0;
	char dev_name[MAX_DEV_NAME_LEN] = {0, };

	ret = addr_check(r_addr);
	if(ret < 0)
	{
        printk(KERN_ERR "[%s] error: addr_check failed at %#llx %x\n",
			__func__, (long long)r_addr ,r_size);
        return -1;
	}

    snprintf(dev_name, MAX_DEV_NAME_LEN, "/dev/block/mmcblk0p%d", openblk);
	printk(KERN_ERR "[%s] dev_name = %s\n",	__func__, dev_name);

	fota_file = filp_open(dev_name, O_RDONLY, 0);
    if (fota_file == NULL) {
        printk(KERN_ERR "[%s] error: open failed at %#llx %x\n",
			 __func__, (long long)r_addr ,r_size);
        return -1;
    }

	_segfs         = get_fs();
	set_fs(get_ds());

	seek_result = fota_file->f_op->llseek(fota_file, r_addr, SEEK_SET);
	if (seek_result != r_addr) {
        printk(KERN_ERR "[%s] error: lseek failed at %llx %#llx %x\n", 
			__func__, seek_result ,(long long)r_addr ,r_size);
        return -1;
    }

	ret = fota_file->f_op->read(fota_file, r_buff, r_size, &fota_file->f_pos);
    if (ret < 0) {
        printk(KERN_ERR "[%s] error: read failed at %d %#llx %x\n",
			__func__, ret ,(long long)r_addr ,r_size);
        return -1;
    }

	set_fs(_segfs);

	filp_close(fota_file, NULL);

	return 0;
}

static int  fota_flash_write ( int openblk, loff_t w_addr,
								 unsigned char *w_buff, size_t w_size )
{
	int ret = 0;
	struct file *fota_file;
	mm_segment_t   _segfs;
	loff_t seek_result = 0;
	char dev_name[MAX_DEV_NAME_LEN] = {0, };

	ret = addr_check(w_addr);
	if(ret < 0)
	{
        printk(KERN_ERR "[%s] error: addr_check failed at %#llx %x\n",
			__func__, (long long)w_addr ,w_size);
        return -1;
	}

    snprintf(dev_name, MAX_DEV_NAME_LEN, "/dev/block/mmcblk0p%d", openblk);
	printk(KERN_ERR "[%s] dev_name = %s\n",	__func__, dev_name);

	fota_file = filp_open(dev_name, O_WRONLY, 0);
    if (IS_ERR(fota_file)) {
        printk(KERN_ERR "[%s] error: open failed. err_code = %lx at %#llx %x\n",
			 __func__, IS_ERR(fota_file), (long long)w_addr ,w_size);
        return -1;
    }

	_segfs         = get_fs();
	set_fs(get_ds());

	seek_result = fota_file->f_op->llseek(fota_file, w_addr, SEEK_SET);
	if (seek_result != w_addr) {
        printk(KERN_ERR "[%s] error: lseek failed at %llx %#llx %x\n", 
			__func__, seek_result ,(long long)w_addr ,w_size);
        return -1;
    }

	ret = fota_file->f_op->write(fota_file, w_buff, w_size, &fota_file->f_pos);
    if (ret < 0) {
        printk(KERN_ERR "[%s] error: write failed at %x %#llx %x\n",
			__func__, ret ,(long long)w_addr ,w_size);
        return -1;
    }

	set_fs(_segfs);

	filp_close(fota_file, NULL);

    return 0;
}


