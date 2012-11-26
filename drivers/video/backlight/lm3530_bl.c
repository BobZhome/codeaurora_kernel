/* drivers/video/backlight/lm3530_bl.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <mach/board.h>
#include <linux/earlysuspend.h>

#define MAX_LEVEL		0x71
#define MIN_LEVEL 		0x05
#define DEFAULT_LEVEL	0x25

#define I2C_BL_NAME "lm3530"

#define BL_ON	1
#define BL_OFF	0

static struct i2c_client *lm3530_i2c_client;

struct backlight_platform_data {
	void (*platform_init)(void);
	int gpio;
	unsigned int mode;
	int max_current;
	int init_on_boot;
	int min_brightness;
	int max_brightness;
	int default_brightness;
};

struct lm3530_device {
	struct i2c_client *client;
	struct backlight_device *bl_dev;
	int gpio;
	int max_current;
	int min_brightness;
	int max_brightness;
	int default_brightness;
	struct mutex bl_mutex;
};

static const struct i2c_device_id lm3530_bl_id[] = {
	{ I2C_BL_NAME, 0 },
	{ },
};

static int lm3530_write_reg(struct i2c_client *client,
		unsigned char reg, unsigned char val);

static int cur_main_lcd_level;
static int saved_main_lcd_level;
static int backlight_status = BL_OFF;

static struct lm3530_device *main_lm3530_dev;
#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend early_suspend;

#if defined(CONFIG_FB_MSM_MIPI_LGD_VIDEO_QHD_PT_PANEL)
static int is_early_suspended = false;
static int requested_in_early_suspend_lcd_level= 0;
#endif  
#endif

#if !defined(CONFIG_FB_MSM_MIPI_LGD_VIDEO_QHD_PT_PANEL)
static struct early_suspend * h;
#endif


static void lm3530_hw_reset(void)
{
	int gpio = main_lm3530_dev->gpio;
	if (gpio_is_valid(gpio)) {
		gpio_direction_output(gpio, 1);
		gpio_set_value_cansleep(gpio, 1);
		/* gpio is defined in board-lgp_s3-panel.c */
		mdelay(1);
	}
}

static int lm3530_write_reg(struct i2c_client *client,
		unsigned char reg, unsigned char val)
{
	u8 buf[2];
	struct i2c_msg msg = {
		client->addr, 0, 2, buf
	};

	buf[0] = reg;
	buf[1] = val;

	if (i2c_transfer(client->adapter, &msg, 1) < 0)
		dev_err(&client->dev, "i2c write error\n");

	return 0;
}

static void lm3530_set_main_current_level(struct i2c_client *client, int level)
{
	struct lm3530_device *dev;
	int cal_value;
#if !defined(CONFIG_MACH_MSM8960_L0)
	int min_brightness 	= main_lm3530_dev->min_brightness;
	int max_brightness 	= main_lm3530_dev->max_brightness;
#endif
	dev = (struct lm3530_device *)i2c_get_clientdata(client);
	if (level == -1)
		level = dev->default_brightness;

	dev->bl_dev->props.brightness = cur_main_lcd_level = level;

	mutex_lock(&main_lm3530_dev->bl_mutex);

#ifdef CONFIG_MACH_MSM8960_L0
	if (level != 0) {
		cal_value = level;
		lm3530_write_reg(client, 0xA0, cal_value);
		/*printk("%s() : cal_value is : 0x%x\n", __func__, cal_value);*/
	} else {
		lm3530_write_reg(client, 0x10, 0x00);
	}
#else
	if (level != 0) {
		if (level <= MIN_LEVEL) {
			cal_value = min_brightness;
		} else if (level > MIN_LEVEL && level <= MAX_LEVEL) {
			cal_value = (max_brightness - min_brightness)*level
						/(MAX_LEVEL - MIN_LEVEL)-
						((max_brightness - min_brightness)*MIN_LEVEL
						/(MAX_LEVEL - MIN_LEVEL) - min_brightness);
		} else if (level > MAX_LEVEL) {
			cal_value = max_brightness;
		}

		lm3530_write_reg(client, 0xA0, cal_value);
		/*printk("%s() :level is : %d, cal_value is : 0x%x\n", __func__,
		 * level, cal_value);*/
	} else {
		lm3530_write_reg(client, 0x10, 0x00);
	}
#endif

	msleep(1);

	mutex_unlock(&main_lm3530_dev->bl_mutex);
}

void lm3530_backlight_on(int level)
{

#if defined(CONFIG_HAS_EARLYSUSPEND) && \
		(defined(CONFIG_FB_MSM_MIPI_LGD_VIDEO_QHD_PT_PANEL))
		if(is_early_suspended)
		{
			requested_in_early_suspend_lcd_level = level;
			return;
		}
#endif /* CONFIG_HAS_EARLYSUSPEND */

	if (backlight_status == BL_OFF) {
		lm3530_hw_reset();

		lm3530_write_reg(main_lm3530_dev->client, 0xA0, 0x00);
		/* reset 0 brightness */
		lm3530_write_reg(main_lm3530_dev->client, 0x10,
				main_lm3530_dev->max_current);
		lm3530_write_reg(main_lm3530_dev->client, 0x30, 0x12);
		/* fade in, out */

		/* msleep(100); */
	}

	/* printk("%s received (prev backlight_status: %s)\n",
	 * __func__, backlight_status?"ON":"OFF");*/
	lm3530_set_main_current_level(main_lm3530_dev->client, level);
	backlight_status = BL_ON;

	return;
}

#if defined(CONFIG_FB_MSM_MIPI_LGD_VIDEO_QHD_PT_PANEL) || \
    !defined(CONFIG_HAS_EARLYSUSPEND)
void lm3530_backlight_off(void)
#else
void lm3530_backlight_off(struct early_suspend * h)
#endif
{
	int gpio = main_lm3530_dev->gpio;
	printk("%s, backlight_status : %d\n",__func__,backlight_status);
	if (backlight_status == BL_OFF)
		return;
	saved_main_lcd_level = cur_main_lcd_level;
	lm3530_set_main_current_level(main_lm3530_dev->client, 0);
	backlight_status = BL_OFF;

	gpio_tlmm_config(GPIO_CFG(gpio, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
				GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_direction_output(gpio, 0);
	msleep(6);
	return;
}

void lm3530_lcd_backlight_set_level(int level)
{
	if (level > MAX_LEVEL)
		level = MAX_LEVEL;

	if (lm3530_i2c_client != NULL) {
		if (level == 0)
#if defined(CONFIG_FB_MSM_MIPI_LGD_VIDEO_QHD_PT_PANEL) || \
    !defined(CONFIG_HAS_EARLYSUSPEND)
			lm3530_backlight_off();
#else
			lm3530_backlight_off(h);
#endif			
		else
			lm3530_backlight_on(level);

		/*printk("%s() : level is : %d\n", __func__, level);*/
	} else{
		printk(KERN_INFO "%s(): No client\n", __func__);
	}
}
EXPORT_SYMBOL(lm3530_lcd_backlight_set_level);

#if defined(CONFIG_HAS_EARLYSUSPEND) && \
	(defined(CONFIG_FB_MSM_MIPI_LGD_VIDEO_QHD_PT_PANEL))
void lm3530_early_suspend(struct early_suspend * h)
{
	is_early_suspended = true;

	pr_info("%s[Start] backlight_status: %d\n", __func__, backlight_status);
	if (backlight_status == BL_OFF)
		return;

	lm3530_lcd_backlight_set_level(0);
}

void lm3530_late_resume(struct early_suspend * h)
{
	is_early_suspended = false;

	pr_info("%s[Start] backlight_status: %d\n", __func__, backlight_status);
	if (backlight_status == BL_ON)
		return;

	lm3530_lcd_backlight_set_level(requested_in_early_suspend_lcd_level);
	return;
}
#endif /* CONFIG_HAS_EARLYSUSPEND */

static int bl_set_intensity(struct backlight_device *bd)
{
	lm3530_lcd_backlight_set_level(bd->props.brightness);
	return 0;
}

static int bl_get_intensity(struct backlight_device *bd)
{
    return cur_main_lcd_level;
}

static ssize_t lcd_backlight_show_level(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int r;

	r = snprintf(buf, PAGE_SIZE, "LCD Backlight Level is : %d\n",
			cur_main_lcd_level);

	return r;
}

static ssize_t lcd_backlight_store_level(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int level;

	if (!count)
		return -EINVAL;

	level = simple_strtoul(buf, NULL, 10);
	lm3530_lcd_backlight_set_level(level);

	return count;
}

static int lm3530_bl_resume(struct i2c_client *client)
{
#if defined(CONFIG_FB_MSM_MIPI_LGD_VIDEO_QHD_PT_PANEL)
		lm3530_lcd_backlight_set_level(saved_main_lcd_level);
#else
		lm3530_backlight_on(saved_main_lcd_level);
#endif
    return 0;
}

static int lm3530_bl_suspend(struct i2c_client *client, pm_message_t state)
{
    printk(KERN_INFO "%s: new state: %d\n", __func__, state.event);


#if defined(CONFIG_FB_MSM_MIPI_LGD_VIDEO_QHD_PT_PANEL) || \
		!defined(CONFIG_HAS_EARLYSUSPEND)
		lm3530_lcd_backlight_set_level(saved_main_lcd_level);
#else
		lm3530_backlight_off(h);
#endif
    return 0;
}

static ssize_t lcd_backlight_show_on_off(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	printk(KERN_INFO "%s received (prev backlight_status: %s)\n", __func__,
			backlight_status ? "ON" : "OFF");
	return 0;
}

static ssize_t lcd_backlight_store_on_off(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int on_off;
	struct i2c_client *client = to_i2c_client(dev);

	if (!count)
		return -EINVAL;

	printk(KERN_INFO "%s received (prev backlight_status: %s)\n", __func__,
			backlight_status ? "ON" : "OFF");

	on_off = simple_strtoul(buf, NULL, 10);

	printk(KERN_ERR "%d", on_off);

	if (on_off == 1)
		lm3530_bl_resume(client);
	else if (on_off == 0)
		lm3530_bl_suspend(client, PMSG_SUSPEND);

	return count;

}
DEVICE_ATTR(lm3530_level, 0664, lcd_backlight_show_level,
		lcd_backlight_store_level);
DEVICE_ATTR(lm3530_backlight_on_off, 0664, lcd_backlight_show_on_off,
		lcd_backlight_store_on_off);

static struct backlight_ops lm3530_bl_ops = {
	.update_status = bl_set_intensity,
	.get_brightness = bl_get_intensity,
};

static int __devinit lm3530_probe(struct i2c_client *i2c_dev,
		const struct i2c_device_id *id)
{
	struct backlight_platform_data *pdata;
	struct lm3530_device *dev;
	struct backlight_device *bl_dev;
	struct backlight_properties props;
	int err;

	printk("%s: i2c probe start\n", __func__);

	pdata = i2c_dev->dev.platform_data;
	lm3530_i2c_client = i2c_dev;

	dev = kzalloc(sizeof(struct lm3530_device), GFP_KERNEL);
	if (dev == NULL) {
		dev_err(&i2c_dev->dev, "fail alloc for lm3530_device\n");
		return 0;
	}

	main_lm3530_dev = dev;

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = MAX_LEVEL;

	bl_dev = backlight_device_register(I2C_BL_NAME, &i2c_dev->dev, NULL,
			&lm3530_bl_ops, &props);
	bl_dev->props.max_brightness = MAX_LEVEL;
	bl_dev->props.brightness = DEFAULT_LEVEL;
	bl_dev->props.power = FB_BLANK_UNBLANK;

	dev->bl_dev = bl_dev;
	dev->client = i2c_dev;
	dev->gpio = pdata->gpio;
	dev->max_current = pdata->max_current;
	dev->min_brightness = pdata->min_brightness;
	dev->default_brightness = pdata->default_brightness;
	dev->max_brightness = pdata->max_brightness;
	i2c_set_clientdata(i2c_dev, dev);

	if (dev->gpio && gpio_request(dev->gpio, "lm3530 reset") != 0)
		return -ENODEV;

	mutex_init(&dev->bl_mutex);

	err = device_create_file(&i2c_dev->dev, &dev_attr_lm3530_level);
	err = device_create_file(&i2c_dev->dev,
			&dev_attr_lm3530_backlight_on_off);

#ifdef CONFIG_HAS_EARLYSUSPEND
#if defined(CONFIG_FB_MSM_MIPI_LGD_VIDEO_QHD_PT_PANEL)
	early_suspend.suspend = lm3530_early_suspend;
	early_suspend.resume = lm3530_late_resume;
#else
	early_suspend.suspend = lm3530_backlight_off;
#endif
	register_early_suspend(&early_suspend);
#endif /* CONFIG_HAS_EARLYSUSPEND */

	return 0;
}

static int __devexit lm3530_remove(struct i2c_client *i2c_dev)
{
	struct lm3530_device *dev;
	int gpio = main_lm3530_dev->gpio;

	device_remove_file(&i2c_dev->dev, &dev_attr_lm3530_level);
	device_remove_file(&i2c_dev->dev, &dev_attr_lm3530_backlight_on_off);
	dev = (struct lm3530_device *)i2c_get_clientdata(i2c_dev);
	backlight_device_unregister(dev->bl_dev);
	i2c_set_clientdata(i2c_dev, NULL);

	if (gpio_is_valid(gpio))
		gpio_free(gpio);
	return 0;
}

static struct i2c_driver main_lm3530_driver = {
	.probe = lm3530_probe,
	.remove = lm3530_remove,
	.suspend = NULL,
	.resume = NULL,
	.id_table = lm3530_bl_id,
	.driver = {
		.name = I2C_BL_NAME,
		.owner = THIS_MODULE,
	},
};


static int __init lcd_backlight_init(void)
{
	static int err;

	err = i2c_add_driver(&main_lm3530_driver);

	return err;
}

module_init(lcd_backlight_init);

MODULE_DESCRIPTION("LM3530 Backlight Control");
MODULE_AUTHOR("Jaeseong Gim <jaeseong.gim@lge.com>");
MODULE_LICENSE("GPL");
