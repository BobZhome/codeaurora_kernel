/*
 *  arch/arm/include/asm/mach/mmc.h
 */
#ifndef ASMARM_MACH_MMC_H
#define ASMARM_MACH_MMC_H

#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio_func.h>

struct embedded_sdio_data {
        struct sdio_cis cis;
        struct sdio_cccr cccr;
        struct sdio_embedded_func *funcs;
        int num_funcs;
};

struct mmc_platform_data {
	unsigned int ocr_mask;			/* available voltages */
	int built_in;				/* built-in device flag */
	int card_present;			/* card detect state */
	u32 (*translate_vdd)(struct device *, unsigned int);
	unsigned int (*status)(struct device *);
	struct embedded_sdio_data *embedded_sdio;
	int (*register_status_notify)(void (*callback)(int card_present, void *dev_id), void *dev_id);
        unsigned long irq_flags;
        unsigned long mmc_bus_width;
        int (*wpswitch) (struct device *);
	int dummy52_required;
	unsigned int msmsdcc_fmin;
	unsigned int msmsdcc_fmid;
	unsigned int msmsdcc_fmax;
	bool nonremovable;
	bool pclk_src_dfab;
	int (*cfg_mpm_sdiowakeup)(struct device *, bool);
};

#endif
