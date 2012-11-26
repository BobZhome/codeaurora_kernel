/* 
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>

#define DIAG_SLATE_F            (246)


int oemmsg_process_pkt(int cmd, int sub_id, int sub_cmd)
{
  if(cmd == DIAG_SLATE_F)
  {
    if((sub_id == 0x31)||(sub_id == 0x25)||(sub_id == 0x37))
    {
      return 1;
    }
  }
  return 0;
}
