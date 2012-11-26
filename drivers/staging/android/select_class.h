/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C)2012 KYOCERA Corporation
 */

#ifndef _LINUX_SELECT_CLASS_H
#define _LINUX_SELECT_CLASS_H

struct select_dev {
  const char *name;

  void (*enable)(struct select_dev *sdev, int timeout);

  int (*get_select)(struct select_dev *sdev);

  struct device *dev;
  int    index;
  int    state;
};

extern int select_dev_register(struct select_dev *dev);
extern void select_dev_unregister(struct select_dev *dev);

#endif
