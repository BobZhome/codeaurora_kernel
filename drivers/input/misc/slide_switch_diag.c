/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
*/








/*****************************************************************************/
/* include files                                                             */
/*****************************************************************************/
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <mach/hs_io_ctl_a.h>
#include <mach/proc_comm_kyocera.h>

/*****************************************************************************/
/* defines                                                                   */
/*****************************************************************************/
#define SLIDESW_DIAG_NAME             "slide_sw_diag"

/*****************************************************************************/
/* static variable                                                           */
/*****************************************************************************/
static struct input_dev *input;

/*****************************************************************************/
/* function                                                                  */
/*****************************************************************************/
static ssize_t
slidesw_status_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    int slide_sts = gpio_get_value(GPIO_SLIDE);

    if(slide_sts != 0 && slide_sts != 1)
    {
      printk("%s:gpio status read error[%d].\n",__func__,slide_sts);
      slide_sts = -1;
    }

    return sprintf(buf, "%d\n", slide_sts);
}

static ssize_t
slidesw_status_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf,
        size_t count)
{
    int slide_sts =  simple_strtol(buf,NULL,0);

    if(slide_sts == 0 || slide_sts == 1)
    {
      printk("%s:slide status setting[%d]\n",__func__,slide_sts);
      input_report_switch(input, SW_LID, slide_sts);
      input_sync(input);
      proc_comm_rpc_apps_to_modem(PROC_COMM_SUB_CMD_SLIDE_STATUS_INFO, &slide_sts);
    }

    return count;
}

static DEVICE_ATTR(status, S_IRUGO|S_IWUSR|S_IWGRP,
                    slidesw_status_show, slidesw_status_store);
static struct attribute *slidesw_attributes[] = {
    &dev_attr_status.attr,
    NULL
};
static struct attribute_group slidesw_attribute_group = {
    .attrs = slidesw_attributes
};

static int
slide_sw_diag_init_input(void)
{
    int err = -1;

    input = input_allocate_device();

    if (input == NULL) {
        printk("%s: input_allocate_device() error[%d]\n",__func__,err);
        return err;
    }

    input->name = SLIDESW_DIAG_NAME;
    input_set_capability(input, EV_SW, SW_LID);
    input_set_drvdata(input, NULL);
    err = input_register_device(input);

    if(err) {
        printk("%s: input_register_device() error[%d]\n",__func__,err);
        return err;
    }

    err = sysfs_create_group(&input->dev.kobj,&slidesw_attribute_group);

    if(err) {
        printk("%s: sysfs_create_group() error[%d]\n",__func__,err);
        return err;
    }

    return err;
}

static int __init
slide_sw_diag_init(void)
{
    return slide_sw_diag_init_input();
}

static void __exit
slide_sw_diag_term(void)
{
}

module_init(slide_sw_diag_init);
module_exit(slide_sw_diag_term);

