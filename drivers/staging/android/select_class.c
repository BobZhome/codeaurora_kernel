/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C)2012 KYOCERA Corporation
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/err.h>

#include "select_class.h"

static struct class *select_class;
static atomic_t device_count;

static ssize_t enable_show(struct device *dev, struct device_attribute *attr,
  char *buf)
{
  struct select_dev *tdev = dev_get_drvdata(dev);
  int remaining = tdev->get_select(tdev);

  return sprintf(buf, "%d\n", remaining);
}

static ssize_t enable_store(struct device *dev, struct device_attribute *attr,
  const char *buf, size_t size)
{
  struct select_dev *tdev = dev_get_drvdata(dev);
  int value;

  if (sscanf(buf, "%d", &value) != 1)
    return -EINVAL;

  tdev->enable(tdev, value);

  return size;
}

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, enable_show, enable_store);

static int create_select_class(void)
{
  if (!select_class) {
    select_class = class_create(THIS_MODULE, "select");
    if (IS_ERR(select_class))
      return PTR_ERR(select_class);
    atomic_set(&device_count, 0);
  }

  return 0;
}

int select_dev_register(struct select_dev *tdev)
{
  int ret;

  if (!tdev || !tdev->name || !tdev->enable || !tdev->get_select)
    return -EINVAL;

  ret = create_select_class();
  if (ret < 0)
    return ret;

  tdev->index = atomic_inc_return(&device_count);
  tdev->dev = device_create(select_class, NULL,
    MKDEV(0, tdev->index), NULL, tdev->name);
  if (IS_ERR(tdev->dev))
    return PTR_ERR(tdev->dev);

  ret = device_create_file(tdev->dev, &dev_attr_enable);
  if (ret < 0)
    goto err_create_file;

  dev_set_drvdata(tdev->dev, tdev);
  tdev->state = 0;
  return 0;

err_create_file:
  device_destroy(select_class, MKDEV(0, tdev->index));
  printk(KERN_ERR "select: Failed to register driver %s\n",
      tdev->name);

  return ret;
}
EXPORT_SYMBOL_GPL(select_dev_register);

void select_dev_unregister(struct select_dev *tdev)
{
  device_remove_file(tdev->dev, &dev_attr_enable);
  device_destroy(select_class, MKDEV(0, tdev->index));
  dev_set_drvdata(tdev->dev, NULL);
}
EXPORT_SYMBOL_GPL(select_dev_unregister);

static int __init select_init(void)
{
  return create_select_class();
}

static void __exit select_exit(void)
{
  class_destroy(select_class);
}

module_init(select_init);
module_exit(select_exit);

MODULE_AUTHOR("kyocera");
MODULE_DESCRIPTION("select class driver");
MODULE_LICENSE("GPL");
