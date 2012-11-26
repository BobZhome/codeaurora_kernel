/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
*/








/*****************************************************************************/
/* include files                                                             */
/*****************************************************************************/
#include <linux/types.h>
#include <asm/uaccess.h>
#include <linux/kernel.h>
#include <asm/atomic.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/mutex.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <mach/hs_io_ctl_a.h>
#include <apds9900.h>


/*****************************************************************************/
/* extern                                                                    */
/*****************************************************************************/
extern ssize_t
light_enable_show(struct device *dev,
        struct device_attribute *attr,
        char *buf);

extern ssize_t
light_enable_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf,
        size_t count);

extern ssize_t
light_delay_show(struct device *dev,
        struct device_attribute *attr,
        char *buf);

extern ssize_t
light_delay_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf,
        size_t count);

extern ssize_t
light_wake_store(struct device *dev,
                                struct device_attribute *attr,
                                const char *buf, size_t count);

extern ssize_t
light_data_show(struct device *dev,
                               struct device_attribute *attr, char *buf);


extern ssize_t 
light_dbglog_show(struct device *dev,
                  struct device_attribute *attr, char *buf);


static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP,
                    light_enable_show, light_enable_store);
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP,
                    light_delay_show, light_delay_store);
static DEVICE_ATTR(wake, S_IWUSR|S_IWGRP, NULL, light_wake_store);
static DEVICE_ATTR(data, S_IRUGO, light_data_show, NULL);

static DEVICE_ATTR(dbglog, S_IRUGO, light_dbglog_show, NULL);


static struct attribute *light_attributes[] = {
    &dev_attr_enable.attr,
    &dev_attr_delay.attr,
    &dev_attr_wake.attr,
    &dev_attr_data.attr,

    &dev_attr_dbglog.attr,

    NULL
};

struct attribute_group light_attribute_group = {
    .attrs = light_attributes
};


int light_create_group(struct input_dev *input)
{
	int err;

    err = sysfs_create_group(&input->dev.kobj,&light_attribute_group);

	return err;
}
