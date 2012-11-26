/* 
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 */

#ifndef _LINUX_DEV_LCD_CLASS_H
#define _LINUX_DEV_LCD_CLASS_H

struct lcd_class_struct {
  const char *name;
  void (*set)(struct lcd_class_struct *sdev, unsigned int *value);
  int  (*get)(struct lcd_class_struct *sdev);

  struct device *dev;
  int    index;
  int    state;
};

extern int dev_lcd_class_register(struct lcd_class_struct *dev);
extern void dev_lcd_class_unregister(struct lcd_class_struct *dev);

#endif /* _LINUX_DEV_LCD_CLASS_H */
