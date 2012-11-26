/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
*/

#ifndef _API_NV_H_
#define _API_NV_H_

#define NV_D_ACCESS_UPDATE_ON  1
#define NV_D_ACCESS_UPDATE_OFF 0

typedef struct
{
	char update_flag;
	char set_flag;
}smem_product_line_d_info_type;

typedef struct
{
  char update_flag;
  char set_flag;
} smem_boot_debug_d_info_type;

typedef struct
{
  smem_product_line_d_info_type product_line_d_info;
  smem_boot_debug_d_info_type   boot_debug_d_info;
} smem_nv_d_access_info_type;

struct nv_d_ioctl_info {
  smem_nv_d_access_info_type nv_d_access_info;
};

#define NV_IOC_MAGIC 0xF9
#define IOCTL_NV_D_INFO_READ_CMD        _IOR(NV_IOC_MAGIC, 1, struct nv_d_ioctl_info)
#define IOCTL_NV_D_INFO_WRITE_CMD       _IOW(NV_IOC_MAGIC, 2, struct nv_d_ioctl_info)

#endif /* _API_NV_H_ */

