
/*
HALL_IC_DRIVER_FOR_MSM9860
*/

#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/sysdev.h>
#include <linux/types.h>
#include <linux/time.h>
#include <linux/version.h>
#include <mach/gpio.h>
#include <linux/input/lge_f200_hall_ic.h>


static struct f200_hall_ic_pdata_type f200_hall_ic_pdata = {
	.s_gpio = GPIO_CRADLE_DETECT_S,
	.n_gpio = GPIO_CRADLE_DETECT_N,
	.s_state = 0,
	.n_state = 0,
	.hall_ic_s_irq = MSM_GPIO_TO_INT(GPIO_CRADLE_DETECT_S),
	.hall_ic_n_irq = MSM_GPIO_TO_INT(GPIO_CRADLE_DETECT_N),
	.irqf_trigger_mode = IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING,
};



static void hall_ic_report_event(struct f200_hall_ic_pdata_type *pdata)
{
	printk("%s\n", __FUNCTION__);

	input_report_switch(pdata->input_dev_hall_ic, SW_FRONT_COVER, pdata->n_state);
	input_sync(pdata->input_dev_hall_ic);

	return;

}

static void hall_ic_get_gpio_state(struct f200_hall_ic_pdata_type *pdata)
{

	printk("%s\n", __FUNCTION__);

	pdata->n_state = (gpio_get_value(pdata->n_gpio)) ? 0 : 1;
	printk("S_STATE : %d\n", pdata->n_state);

	return;
}

static void hall_ic_detect_work_func(struct work_struct *work)
{
	struct f200_hall_ic_pdata_type *pdata = container_of(work, struct f200_hall_ic_pdata_type, hall_ic_delayed_work.work);

	printk("%s\n", __FUNCTION__);

	hall_ic_get_gpio_state(pdata);
	hall_ic_report_event(pdata);

	return;
}

static irqreturn_t hall_ic_irq_handler(int irq, void *dev_id)
{
	struct f200_hall_ic_pdata_type *pdata = (struct f200_hall_ic_pdata_type *)dev_id;

	printk("%s\n", __FUNCTION__);

	disable_irq_nosync(pdata->hall_ic_n_irq);

	schedule_delayed_work(&pdata->hall_ic_delayed_work, msecs_to_jiffies(100));

	enable_irq(pdata->hall_ic_n_irq);


	return IRQ_HANDLED;

}

static int f200_hall_ic_probe(struct platform_device *pdev)
{
	struct f200_hall_ic_pdata_type *pdata = pdev->dev.platform_data;

	int ret = 0;

	printk("%s\n", __FUNCTION__);

	gpio_request(pdata->n_gpio, "GPIO_CRADLE_DETECT_N");
	gpio_direction_input(pdata->n_gpio);


	pdata->input_dev_hall_ic= input_allocate_device();

	pdata->input_dev_hall_ic->name = pdev->name;

	set_bit(EV_SYN, pdata->input_dev_hall_ic->evbit);
	set_bit(EV_SW, pdata->input_dev_hall_ic->evbit);
	set_bit(SW_FRONT_COVER, pdata->input_dev_hall_ic->swbit);

	pdata->input_dev_hall_ic->name = "f200_hall_ic";


	ret = input_register_device(pdata->input_dev_hall_ic);

	INIT_DELAYED_WORK(&pdata->hall_ic_delayed_work, hall_ic_detect_work_func);


	ret = request_irq(pdata->hall_ic_n_irq, hall_ic_irq_handler,
						pdata->irqf_trigger_mode, pdev->name, pdata);
	//enable_irq_wake(pdata->hall_ic_n_irq);

	if(ret)
	{
		printk(KERN_ERR "[HALL_IC] interrupt registeration fail! (err:%d)\n", ret);
		return ret;
	}

	schedule_delayed_work(&pdata->hall_ic_delayed_work, 0);

	return 0;
}

static int f200_hall_ic_remove(struct platform_device *pdev)
{

    struct f200_hall_ic_pdata_type *pdata = platform_get_drvdata(pdev);

	printk(KERN_ERR "[HALL_IC] remove (err:%d)\n", 103);

	input_unregister_device(pdata->input_dev_hall_ic);

	pdata = NULL;

	return 0;
}

static int f200_hall_ic_suspend(struct platform_device *pdev, pm_message_t state)
{
	int ret = 0;
    struct f200_hall_ic_pdata_type *pdata = pdev->dev.platform_data;

	printk("%s\n", __FUNCTION__);
	ret = enable_irq_wake(pdata->hall_ic_n_irq);
	if(ret)
	{
		printk(KERN_ERR "[HALL_IC] interrupt wakeup enable fail! (err:%d)\n", ret);
		return ret;
	}
	return 0;
}

static int f200_hall_ic_resume(struct platform_device *pdev)
{
	int ret = 0;
	struct f200_hall_ic_pdata_type *pdata = pdev->dev.platform_data;

	printk("%s\n", __FUNCTION__);
	ret = disable_irq_wake(pdata->hall_ic_n_irq);
	if(ret)
	{
		printk(KERN_ERR "[HALL_IC] interrupt wakeup disable fail! (err:%d)\n", ret);
		return ret;
	}
	schedule_delayed_work(&pdata->hall_ic_delayed_work, msecs_to_jiffies(100));

	return 0;
}

static struct platform_driver f200_hall_ic_driver = {
	.probe		= f200_hall_ic_probe,
	.remove		= f200_hall_ic_remove,
	.suspend		= f200_hall_ic_suspend,
	.resume		= f200_hall_ic_resume,
	.driver		= {
		.name	= "f200_hall_ic",
		.owner	= THIS_MODULE,
	},
};
static struct platform_device *hall_ic_platform_device = NULL;

static int __init f200_hall_ic_init(void)
{
	int ret=0;
	printk("%s\n", __FUNCTION__);

	ret=platform_driver_register(&f200_hall_ic_driver);

	if(ret)
		printk("%s : init fail\n" ,__FUNCTION__);

	hall_ic_platform_device = platform_device_alloc("f200_hall_ic", -1);

	hall_ic_platform_device->dev.platform_data = &f200_hall_ic_pdata;

	ret = platform_device_add(hall_ic_platform_device);

	if(ret)
		printk("%s : init fail\n" ,__FUNCTION__);

	return ret;
}

static void __exit f200_hall_ic_exit(void)
{
	printk("%s\n", __FUNCTION__);

	platform_driver_unregister(&f200_hall_ic_driver);
}

module_init(f200_hall_ic_init);
module_exit(f200_hall_ic_exit);

MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("HALL IC Detection Driver");
MODULE_LICENSE("GPL");


