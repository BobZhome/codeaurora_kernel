/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
*/

#ifndef _CHK_SD_STATE_H_
#define _CHK_SD_STATE_H_

struct sd_status_ioctl_info {
	int            card_sts;
    int            card_total;
    int            card_space;
	unsigned int   manfid;
    int            reserve;
};

#define IOC_MAGIC 0xF8
#define IOCTL_SD_STATUS_READ_CMD          _IOR(IOC_MAGIC, 1, struct sd_status_ioctl_info)
#define IOCTL_SD_SIZE_GET_CMD             _IOR(IOC_MAGIC, 2, struct sd_status_ioctl_info)
#define IOCTL_EMMC_NAND_DEVCODE_GET_CMD   _IOR(IOC_MAGIC, 3, struct sd_status_ioctl_info)

#define SD_CARD_IN  1
#define SD_CARD_OUT 0

#define EMMC_INDEX  0
#define WIFI_INDEX  1
#define SD_INDEX    2

#endif /* _SMEM_FOTA_H_ */
