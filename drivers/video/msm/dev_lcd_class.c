/* 
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/err.h>

#include "dev_lcd_class.h"

extern unsigned int gamma_A[24];
extern unsigned int gamma_B[24];
extern unsigned int gamma_C[24];
static void gammaA_set(struct lcd_class_struct *dev, unsigned int *value)
{
  printk("%s()[0xC8]-------------------\n", __func__);
  printk("00[%x]01[%x]02[%x]03[%x]04[%x]05[%x]\n", value[ 0], value[ 1], value[ 2], value[ 3], value[ 4], value[ 5]);
  printk("06[%x]07[%x]08[%x]09[%x]10[%x]11[%x]\n", value[ 6], value[ 7], value[ 8], value[ 9], value[10], value[11]);
  printk("12[%x]13[%x]14[%x]15[%x]16[%x]17[%x]\n", value[12], value[13], value[14], value[15], value[16], value[17]);
  printk("18[%x]19[%x]20[%x]21[%x]22[%x]23[%x]\n", value[18], value[19], value[20], value[21], value[22], value[23]);
  memcpy(gamma_A, value, sizeof(gamma_A));
}

static int gammaA_get(struct lcd_class_struct *dev)
{
  return(0);
}

static struct lcd_class_struct gammaA = {
  .name = "gammaA",
  .set  = gammaA_set,
  .get  = gammaA_get,
};

static void gammaB_set(struct lcd_class_struct *dev, unsigned int *value)
{
  memcpy(gamma_B, value, sizeof(gamma_B));
}

static int gammaB_get(struct lcd_class_struct *dev)
{
  return(0);
}

static struct lcd_class_struct gammaB = {
  .name = "gammaB",
  .set  = gammaB_set,
  .get  = gammaB_get,
};

static void gammaC_set(struct lcd_class_struct *dev, unsigned int *value)
{
  memcpy(gamma_C, value, sizeof(gamma_C));
}

static int gammaC_get(struct lcd_class_struct *dev)
{
  return(0);
}

static struct lcd_class_struct gammaC = {
  .name = "gammaC",
  .set  = gammaC_set,
  .get  = gammaC_get,
};

/*********************************************************/

static struct class *lcd_dbg_class;
static atomic_t device_count;

static ssize_t enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  struct lcd_class_struct *tdev = dev_get_drvdata(dev);
  int remaining = tdev->get(tdev);

  return sprintf(buf, "%d\n", remaining);
}

static ssize_t enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
  struct lcd_class_struct *tdev = dev_get_drvdata(dev);
  unsigned int work[24] = {0};
  int ret;

  ret = sscanf(buf, "%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,",
               &work[ 0],&work[ 1],&work[ 2],&work[ 3],&work[ 4],&work[ 5],&work[ 6],&work[ 7],
               &work[ 8],&work[ 9],&work[10],&work[11],&work[12],&work[13],&work[14],&work[15],
               &work[16],&work[17],&work[18],&work[19],&work[20],&work[21],&work[22],&work[23]);
  if(ret < 0){
    return -EINVAL;
  }
  tdev->set(tdev, work);

  return size;
}

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, enable_show, enable_store);

static int create_dev_lcd_class(void)
{
  if (!lcd_dbg_class){
    lcd_dbg_class = class_create(THIS_MODULE, "dev_lcd");
    if (IS_ERR(lcd_dbg_class)){
      return PTR_ERR(lcd_dbg_class);
    }
    atomic_set(&device_count, 0);
  }

  dev_lcd_class_register(&gammaA);
  dev_lcd_class_register(&gammaB);
  dev_lcd_class_register(&gammaC);
  return 0;
}

int dev_lcd_class_register(struct lcd_class_struct *tdev)
{
  int ret;

  if (!tdev || !tdev->name || !tdev->set || !tdev->get){
    return -EINVAL;
  }

  tdev->index = atomic_inc_return(&device_count);
  tdev->dev = device_create(lcd_dbg_class, NULL, MKDEV(0, tdev->index), NULL, tdev->name);
  if (IS_ERR(tdev->dev)){
    return PTR_ERR(tdev->dev);
  }

  ret = device_create_file(tdev->dev, &dev_attr_enable);
  if (ret < 0){
    goto err_create_file;
  }

  dev_set_drvdata(tdev->dev, tdev);
  tdev->state = 0;
  return 0;

err_create_file:
  device_destroy(lcd_dbg_class, MKDEV(0, tdev->index));
  printk(KERN_ERR "dev lcd: Failed to register driver %s\n", tdev->name);

  return ret;
}
EXPORT_SYMBOL_GPL(dev_lcd_class_register);

void dev_lcd_class_unregister(struct lcd_class_struct *tdev)
{
  device_remove_file(tdev->dev, &dev_attr_enable);
  device_destroy(lcd_dbg_class, MKDEV(0, tdev->index));
  dev_set_drvdata(tdev->dev, NULL);
}
EXPORT_SYMBOL_GPL(dev_lcd_class_unregister);

static int __init dev_lcd_class_init(void)
{
  return create_dev_lcd_class();
}

static void __exit dev_lcd_class_exit(void)
{
  class_destroy(lcd_dbg_class);
}

module_init(dev_lcd_class_init);
module_exit(dev_lcd_class_exit);

MODULE_AUTHOR("kyocera");
MODULE_DESCRIPTION("dev lcd class driver");
MODULE_LICENSE("GPL");
