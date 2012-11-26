/*
 * Monkey Device Module -- Provides a monkey support
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2011 KYOCERA Corporation
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include "monkey_device.h"

#define DEBUG
#undef DEBUG

#ifdef DEBUG
#define D_DUMP_BUFFER(prestr, cnt, buf) \
print_hex_dump(KERN_DEBUG, prestr, 16, 1, DUMP_PREFIX_ADDRESS, buf, cnt, 1);
#else
#define D_DUMP_BUFFER(prestr, cnt, buf) do {} while (0)
#endif

#ifdef DEBUG
#define DBG_KERNEL(x...) printk(x)
#else
#define DBG_KERNEL(x...) do {} while (0)
#endif

#define MONKEY_DEVICE_BASE_ADDR   (0xFB4FF400)

#define MONKEY_DEVICE_MAGIC_ADDR  (MONKEY_DEVICE_BASE_ADDR)
#define MONKEY_DEVICE_STOP_ADDR   (MONKEY_DEVICE_MAGIC_ADDR+MONKEY_DEVICE_MAGIC_SIZE)
#define MONKEY_DEVICE_CYCLE_ADDR  (MONKEY_DEVICE_STOP_ADDR+MONKEY_DEVICE_STOP_SIZE)
#define MONKEY_DEVICE_LOG_ADDR    (MONKEY_DEVICE_CYCLE_ADDR+MONKEY_DEVICE_CYCLE_SIZE)
#define MONKEY_DEVICE_FILE_ADDR   (MONKEY_DEVICE_LOG_ADDR+MONKEY_DEVICE_LOG_SIZE)

static char *g_monkey_magic = (char *)MONKEY_DEVICE_MAGIC_ADDR;
static int *g_monkey_stop   = (int *)MONKEY_DEVICE_STOP_ADDR;
static int *g_monkey_cycle = (int *)MONKEY_DEVICE_CYCLE_ADDR;
static char *g_monkey_log  = (char *)MONKEY_DEVICE_LOG_ADDR;
static char *g_monkey_file = (char *)MONKEY_DEVICE_FILE_ADDR;

spinlock_t g_monkey_lock;

long monkey_device_ioctl(
/* struct inode *inode, */
		    struct file *file,
		    unsigned int cmd,
		    unsigned long arg)
{
	int ret = 0;

	DBG_KERNEL(KERN_INFO "monkey_device_ioctl[0x%x]\n", cmd);
	
	spin_lock(&g_monkey_lock);
            
	switch (cmd) {
		
	case MONKEY_DEVICE_IOCTL_SET_MAGIC:
		memcpy( g_monkey_magic, MONKEY_DEVICE_MAGCI_VALUE, MONKEY_DEVICE_MAGIC_SIZE );
		D_DUMP_BUFFER("SetMagic: ", MONKEY_DEVICE_MAGIC_SIZE, g_monkey_magic);
		break;
		
	case MONKEY_DEVICE_IOCTL_RESET_MAGIC:
		memset( g_monkey_magic, 0x00, MONKEY_DEVICE_MAGIC_SIZE);
		D_DUMP_BUFFER("ResetMagic: ", MONKEY_DEVICE_MAGIC_SIZE, g_monkey_magic);
		break;
		
	case MONKEY_DEVICE_IOCTL_GET_MAGIC:
		D_DUMP_BUFFER("GetMagic: ", MONKEY_DEVICE_MAGIC_SIZE, g_monkey_magic);
		if( copy_to_user( (void *)arg, (void *)g_monkey_magic , MONKEY_DEVICE_MAGIC_SIZE)) {
			ret = -1;
		}
		break;
		
	case MONKEY_DEVICE_IOCTL_SET_STOP:
		*g_monkey_stop = (int)arg;
		DBG_KERNEL(KERN_INFO "Set stop [%d]\n", *g_monkey_stop);
		break;
		
	case MONKEY_DEVICE_IOCTL_GET_STOP:
		if( copy_to_user( (void *)arg, (void *)g_monkey_stop , MONKEY_DEVICE_STOP_SIZE)) {
			ret = -1;
		}
		DBG_KERNEL(KERN_INFO "Get stop [%d]\n", *g_monkey_stop);
		break;
		
	case MONKEY_DEVICE_IOCTL_SET_CYCLE:
		*g_monkey_cycle = (int)arg;
		DBG_KERNEL(KERN_INFO "Set cycle [%d]\n", *g_monkey_cycle);
		break;
		
	case MONKEY_DEVICE_IOCTL_GET_CYCLE:
		if( copy_to_user( (void *)arg, (void *)g_monkey_cycle , MONKEY_DEVICE_CYCLE_SIZE)) {
			ret = -1;
		}
		DBG_KERNEL(KERN_INFO "Get cycle [%d]\n", *g_monkey_cycle);
		break;
		
	case MONKEY_DEVICE_IOCTL_SET_LOG:
		if( copy_from_user( (void *)g_monkey_log, (void *)arg , MONKEY_DEVICE_LOG_SIZE)) {
			ret = -1;
		}
		D_DUMP_BUFFER("SetLog: ", strlen(g_monkey_log), g_monkey_log);
		break;
		
	case MONKEY_DEVICE_IOCTL_GET_LOG:
		if( copy_to_user( (void *)arg, (void *)g_monkey_log , MONKEY_DEVICE_LOG_SIZE)) {
			ret = -1;
		}
		D_DUMP_BUFFER("GetLog: ", strlen(g_monkey_log), g_monkey_log);
		break;
		
	case MONKEY_DEVICE_IOCTL_SET_FILE:
		if( copy_from_user( (void *)g_monkey_file, (void *)arg , MONKEY_DEVICE_FILE_SIZE)) {
			ret = -1;
		}
		D_DUMP_BUFFER("SetFileName: ", strlen(g_monkey_file), g_monkey_file);
		break;
		
	case MONKEY_DEVICE_IOCTL_GET_FILE:
		if( copy_to_user( (void *)arg, (void *)g_monkey_file , MONKEY_DEVICE_FILE_SIZE)) {
			ret = -1;
		}
		D_DUMP_BUFFER("GetFileName: ", strlen(g_monkey_file), g_monkey_file);
		break;
		
	default:
		ret = -1;
	}
	
	spin_unlock(&g_monkey_lock);

	return ret;
}

static const struct file_operations monkey_device_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = monkey_device_ioctl,
};

static struct miscdevice monkey_miscdev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name  = "monkey",
    .fops  = &monkey_device_fops,
};

static int __init monkey_device_init(void)
{
	int ret;
	
	ret = misc_register( &monkey_miscdev );

	if( ret ) {
		printk(KERN_INFO "Monkey Device Module Initialize Failed!!\n");
		return ret;
	}
	
    spin_lock_init(&g_monkey_lock);

	DBG_KERNEL(KERN_INFO "g_monkey_magic = 0x%p\n", g_monkey_magic);
	DBG_KERNEL(KERN_INFO "g_monkey_stop = 0x%p\n", g_monkey_stop);
	DBG_KERNEL(KERN_INFO "g_monkey_cycle = 0x%p\n", g_monkey_cycle);
	DBG_KERNEL(KERN_INFO "g_monkey_log = 0x%p\n", g_monkey_log);
	DBG_KERNEL(KERN_INFO "g_monkey_file = 0x%p\n", g_monkey_file);

	printk(KERN_INFO "Monkey Device Module Loaded.\n");
	return 0;
}

static void __exit monkey_device_cleanup(void)
{
	misc_deregister( &monkey_miscdev );
	printk(KERN_INFO "Monkey Device Module Unloaded.\n");
}

module_init(monkey_device_init);
module_exit(monkey_device_cleanup);

MODULE_DESCRIPTION("Monkey Device Module");
MODULE_LICENSE("GPL v2");
