/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
*/

#ifndef __MMC_FOTA_H
#define __MMC_FOTA_H


struct mmc_fota_ioctl_info {
    int    openblk;
	loff_t  addr;
	char   *buff;
	size_t size;
	int    result;
};

#define IOC_MAGIC 0xF6
#define IOCTL_MMC_FOTA_INIT_CMD  _IOR(IOC_MAGIC, 1, struct mmc_fota_ioctl_info)
#define IOCTL_MMC_FOTA_READ_CMD  _IOR(IOC_MAGIC, 2, struct mmc_fota_ioctl_info)
#define IOCTL_MMC_FOTA_WRITE_CMD _IOW(IOC_MAGIC, 3, struct mmc_fota_ioctl_info)

#endif /* __MMC_FOTA_H */
