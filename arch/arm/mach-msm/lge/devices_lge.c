/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/i2c/sx150x.h>
#include <linux/gpio.h>
#include <linux/usb/android_composite.h>
#include <linux/msm_ssbi.h>
#include <linux/regulator/gpio-regulator.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/slimbus/slimbus.h>
#include <linux/bootmem.h>
#include <linux/msm_kgsl.h>
#ifdef CONFIG_ANDROID_PMEM
#include <linux/android_pmem.h>
#endif
#include <linux/cyttsp.h>
#include <linux/dma-mapping.h>
#include <linux/platform_data/qcom_crypto_device.h>
#include <linux/platform_data/qcom_wcnss_device.h>
#include <linux/leds.h>
#include <linux/leds-pm8xxx.h>
#include <linux/msm_tsens.h>
#include <linux/memory.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>
#include <asm/hardware/gic.h>
#include <asm/mach/mmc.h>

#include <mach/board.h>
#include <mach/msm_iomap.h>
#include <mach/msm_spi.h>
#ifdef CONFIG_USB_MSM_OTG_72K
#include <mach/msm_hsusb.h>
#else
#include <linux/usb/msm_hsusb.h>
#endif
#include <mach/usbdiag.h>
#include <mach/socinfo.h>
#include <mach/rpm.h>
#include <mach/gpio.h>
#include <mach/gpiomux.h>
#include <mach/msm_bus_board.h>
#include <mach/msm_memtypes.h>
#include <mach/dma.h>
#include <mach/msm_dsps.h>
#include <mach/msm_xo.h>
#include "pm.h"
#include <mach/cpuidle.h>

#ifdef CONFIG_WCD9310_CODEC
#include <linux/slimbus/slimbus.h>
#include <linux/mfd/wcd9xxx/core.h>
#include <linux/mfd/wcd9xxx/pdata.h>
#endif

#include <linux/ion.h>
#include <mach/ion.h>

#include <mach/board_lge.h>

#include "timer.h"
#include "devices.h"
#include "devices-msm8x60.h"
#include "spm.h"
#include "rpm_resources.h"
#include "rpm_log.h"
#include "msm_watchdog.h"

/* lge gpio i2c device */
#define MAX_GPIO_I2C_DEV_NUM     20
#define LOWEST_GPIO_I2C_BUS_NUM	 0
#define MSM8960_I2C_DEV_NUM_MAX     20
#define MSM8960_I2C_DEV_NUM_START   0

static int msm_i2c_dev_num __initdata;
static struct i2c_registry* msm_i2c_devices[MSM8960_I2C_DEV_NUM_MAX] __initdata;

static int gpio_i2c_dev_num __initdata;
static gpio_i2c_init_func_t *i2c_init_func[MAX_GPIO_I2C_DEV_NUM] __initdata;

/* BEGIN: kidong0420.kim@lge.com 2011-11-09 Implement cable detection */
#ifdef CONFIG_LGE_PM
/* LGE_CHANGE
 * Implement cable detection
 * 2011-11-09, kidong0420.kim@lge.com
 */
#include CONFIG_BOARD_HEADER_FILE
#endif

/* BEGIN: kidong0420.kim@lge.com 2011-11-09 Implement cable detection */
#ifdef CONFIG_LGE_PM
/* LGE_CHANGE
 * Implement cable detection
 * 2011-11-09, kidong0420.kim@lge.com
 */
struct chg_cable_info_table {
	int threshhold;
	acc_cable_type type;
	unsigned ta_ma;
	unsigned usb_ma;
};

static struct chg_cable_info_table pm8921_acc_cable_type_data[]={
	{ADC_NO_INIT_CABLE, NO_INIT_CABLE,  C_NO_INIT_TA_MA,    C_NO_INIT_USB_MA},
	{ADC_CABLE_MHL_1K,  CABLE_MHL_1K,   C_MHL_1K_TA_MA,     C_MHL_1K_USB_MA},
	{ADC_CABLE_U_28P7K, CABLE_U_28P7K,  C_U_28P7K_TA_MA,    C_U_28P7K_USB_MA},
	{ADC_CABLE_28P7K,   CABLE_28P7K,    C_28P7K_TA_MA,      C_28P7K_USB_MA},
	{ADC_CABLE_56K,     CABLE_56K,      C_56K_TA_MA,        C_56K_USB_MA},
	{ADC_CABLE_100K,    CABLE_100K,     C_100K_TA_MA,       C_100K_USB_MA},
	{ADC_CABLE_130K,    CABLE_130K,     C_130K_TA_MA,       C_130K_USB_MA},
	{ADC_CABLE_180K,    CABLE_180K,     C_180K_TA_MA,       C_180K_USB_MA},
	{ADC_CABLE_200K,    CABLE_200K,     C_200K_TA_MA,       C_200K_USB_MA},
	{ADC_CABLE_220K,    CABLE_220K,     C_220K_TA_MA,       C_220K_USB_MA},
	{ADC_CABLE_270K,    CABLE_270K,     C_270K_TA_MA,       C_270K_USB_MA},
	{ADC_CABLE_330K,    CABLE_330K,     C_330K_TA_MA,       C_330K_USB_MA},
	{ADC_CABLE_620K,    CABLE_620K,     C_620K_TA_MA,       C_620K_USB_MA},
	{ADC_CABLE_910K,    CABLE_910K,     C_910K_TA_MA,       C_910K_USB_MA},
	{ADC_CABLE_NONE,    CABLE_NONE,     C_NONE_TA_MA,       C_NONE_USB_MA},
};

/* This table is only for D1LA(Rev.A/B), D1LV(Rev.A/B/C)*/
static struct chg_cable_info_table pm8921_acc_cable_type_data2[]={
	{ADC_NO_INIT_CABLE2, NO_INIT_CABLE,  C_NO_INIT_TA_MA,    C_NO_INIT_USB_MA},
	{ADC_CABLE_MHL_1K2,  CABLE_MHL_1K,   C_MHL_1K_TA_MA,     C_MHL_1K_USB_MA},
	{ADC_CABLE_U_28P7K2, CABLE_U_28P7K,  C_U_28P7K_TA_MA,    C_U_28P7K_USB_MA},
	{ADC_CABLE_28P7K2,   CABLE_28P7K,    C_28P7K_TA_MA,      C_28P7K_USB_MA},
	{ADC_CABLE_56K2,     CABLE_56K,      C_56K_TA_MA,        C_56K_USB_MA},
	{ADC_CABLE_100K2,    CABLE_100K,     C_100K_TA_MA,       C_100K_USB_MA},
	{ADC_CABLE_130K2,    CABLE_130K,     C_130K_TA_MA,       C_130K_USB_MA},
	{ADC_CABLE_180K2,    CABLE_180K,     C_180K_TA_MA,       C_180K_USB_MA},
	{ADC_CABLE_200K2,    CABLE_200K,     C_200K_TA_MA,       C_200K_USB_MA},
	{ADC_CABLE_220K2,    CABLE_220K,     C_220K_TA_MA,       C_220K_USB_MA},
	{ADC_CABLE_270K2,    CABLE_270K,     C_270K_TA_MA,       C_270K_USB_MA},
	{ADC_CABLE_330K2,    CABLE_330K,     C_330K_TA_MA,       C_330K_USB_MA},
	{ADC_CABLE_620K2,    CABLE_620K,     C_620K_TA_MA,       C_620K_USB_MA},
	{ADC_CABLE_910K2,    CABLE_910K,     C_910K_TA_MA,       C_910K_USB_MA},
	{ADC_CABLE_NONE2,    CABLE_NONE,     C_NONE_TA_MA,       C_NONE_USB_MA},
};

#endif

/* 2011-11-15 taew00k.kang@lge.com 1Seg GSBI10 SPI porting [Start] */
#if defined(CONFIG_LGE_BROADCAST_1SEG) || defined(CONFIG_LGE_BROADCAST_TDMB)
#define MSM_GSBI10_PHYS		0x1A200000
#define MSM_GSBI10_QUP_PHYS	(MSM_GSBI10_PHYS + 0x80000)
#endif
/* 2011-11-15 taew00k.kang@lge.com 1Seg GSBI10 SPI porting [End] */

/* for board revision */
static hw_rev_type lge_bd_rev = HW_REV_B;

static int __init board_revno_setup(char *rev_info)
{
	/* CAUTION: These strings are come from LK. */
	char *rev_str[] = {"evb1", "evb2", "rev_a", "rev_b", "rev_c", "rev_d",
		"rev_e", "rev_f", "rev_g", "rev_h", "rev_10", "rev_11", "rev_12",
		"rev_13", "rev_14", "rev_15", "rev_16", "rev_17", "rev_18", "rev_19",
		"revserved"};
	int i;

	for(i=0; i< HW_REV_MAX; i++)
		if( !strncmp(rev_info, rev_str[i], 6)) {
			lge_bd_rev = (hw_rev_type) i;
			system_rev = lge_bd_rev;
			break;
		}

	printk(KERN_INFO "BOARD : LGE %s \n", rev_str[lge_bd_rev]);
	return 1;
}
__setup("lge.rev=", board_revno_setup);

hw_rev_type lge_get_board_revno(void)
{
    return lge_bd_rev;
}

static int usefsa8008 = -1;
bool lge_get_board_usembhc(void)
{
	/*
	* 2011-12-02, junday.lee@lge.com
	* Distinguish between fsa8008 and MBHC
	* using fsa8008 gpio GPIO_EAR_SENSE_N &GPIO_EARPOL_DETECT.
	* Read only once.
	*/
	if(usefsa8008 < 0 )
		usefsa8008 = gpio_get_value_cansleep(GPIO_EAR_SENSE_N) | gpio_get_value_cansleep(GPIO_EARPOL_DETECT);

#ifdef CONFIG_SWITCH_FSA8008

#if defined(CONFIG_MACH_MSM8960_D1LV)
	if (lge_get_board_revno() < HW_REV_D)
		return true;  //MBHC
	else //use fsa8008 from Rev D
		return false; //FSA8008
#endif

#if defined(CONFIG_MACH_MSM8960_D1LA)
	if (lge_get_board_revno() > HW_REV_C) //from Rev.D
		return true;  //MBHC
	else if(lge_get_board_revno() == HW_REV_C) //if rev.C use fsa8008 & MBHC
	{
		if(usefsa8008) //Can distinguish between fsa8008 and MBHC using usefsa8008 value
			return false; //FSA8008
		else
			return true; //MBHC
	}
	else
		return false; //FSA8008
#endif


#if defined(CONFIG_MACH_MSM8960_D1LU)
	return false; //FSA8008
#endif

#if defined(CONFIG_MACH_MSM8960_D1LKT)
	return false; //FSA8008
#endif

#if defined(CONFIG_MACH_MSM8960_D1LSK)
	return false; //FSA8008
#endif

#if defined(CONFIG_MACH_MSM8960_L1sk)
	return false; //FSA8008
#endif

#if defined(CONFIG_MACH_MSM8960_L1kt)
	return false; //FSA8008
#endif

#if defined(CONFIG_MACH_MSM8960_L1u)
	return false; //FSA8008
#endif

#if defined(CONFIG_MACH_MSM8960_L_DCM)
	if (lge_get_board_revno() > HW_REV_A)
		return false; //FSA8008 from rev.B
	else
		return true;  //MBHC rev.A
#endif

//[AUDIO_BSP]_START, 20120305, jeremy.pi@lge.com, Headset detection depends on H/W Revision.
#if defined(CONFIG_MACH_MSM8960_L1A)
	if (lge_get_board_revno() < HW_REV_B)
		return true;  //MBHC
	else
		return false; //FSA8008
#endif
//[AUDIO_BSP]_END, 20120305, jeremy.pi@lge.com

#if defined(CONFIG_MACH_MSM8960_L0)
	if (lge_get_board_revno() >= HW_REV_D) //from Rev.D
		return false;  //FSA8008
	else if(usefsa8008) //Can distinguish between fsa8008 and MBHC using usefsa8008 value
		return false; //FSA8008
	else
		return true; //MBHC
#endif

#if defined(CONFIG_MACH_MSM8960_L2S)
	return false; //FSA8008
#endif

#if defined(CONFIG_MACH_MSM8960_L1m)
	return false; //FSA8008
#endif

#if defined(CONFIG_MACH_MSM8960_L1v)
	return false; //FSA8008
#endif

	return false; //FSA8008(default)

#else
	return true; //MBHC
#endif

}

#ifdef CONFIG_LGE_PM
/* LGE_CHANGE
 * Implement cable detection
 * 2011-11-09, kidong0420.kim@lge.com
 */
int lge_pm_get_cable_info(struct chg_cable_info *cable_info)
{
	char *type_str[] = {"NOT INIT", "MHL 1K", "U_28P7K", "28P7K", "56K",
		"100K", "130K", "180K", "200K", "220K", "270K", "330K", "620K", "910K",
		"OPEN"};

	struct pm8xxx_adc_chan_result result;
	struct chg_cable_info *info = cable_info;
	struct chg_cable_info_table *table;
	int table_size = ARRAY_SIZE(pm8921_acc_cable_type_data);
	int acc_read_value = 0;
	int i, rc;
	int count = 5;

	if (!info) {
		pr_err("lge_pm_get_cable_info: invalid info parameters\n");
		return -1;
	}

	for (i = 0; i < count; i++) {
		rc = pm8xxx_adc_mpp_config_read(PM8XXX_AMUX_MPP_12,
				ADC_MPP_1_AMUX6, &result);

		if (rc < 0) {
			if (rc == -ETIMEDOUT) {
				/* reason: adc read timeout, assume it is open cable */
				info->cable_type = CABLE_NONE;
				info->ta_ma = C_NONE_TA_MA;
				info->usb_ma = C_NONE_USB_MA;
			}
			pr_err("lge_pm_get_cable_info: adc read error - %d\n", rc);
			return rc;
		}

		acc_read_value = (int)result.physical;
		mdelay(10);
	}

	pr_info("%s: acc_read_value - %d\n", __func__, (int)result.physical);
	info->cable_type = NO_INIT_CABLE;
	info->ta_ma = C_NO_INIT_TA_MA;
	info->usb_ma = C_NO_INIT_USB_MA;

	/* assume: adc value must be existed in ascending order */
	for (i = 0; i < table_size; i++) {
		if (lge_get_board_revno() > ADC_CHANGE_REV)
			table = &pm8921_acc_cable_type_data[i];
		else
			table = &pm8921_acc_cable_type_data2[i];

		if (acc_read_value <= table->threshhold) {
			info->cable_type = table->type;
			info->ta_ma = table->ta_ma;
			info->usb_ma = table->usb_ma;

			
			/* LGE_CHANGE
			 * add field for debugging. 
			 * 2012-06-21 lee.yonggu@lge.com
			 */
			info->adc = acc_read_value;
			info->threshould = table->threshhold;
			break;
		}
	}

#ifdef CONFIG_SII8334_MHL_TX
	/* specific case: MHL gender */
	if (GetMHLConnectedStatus()) {
		info->cable_type = CABLE_MHL_1K;
		info->ta_ma = C_MHL_1K_TA_MA;
		info->usb_ma = C_MHL_1K_USB_MA;
	}
#endif

	pr_info("\n\n[PM]Cable detected: %d(%s)(%d, %d)\n\n",
			acc_read_value, type_str[info->cable_type],
			info->ta_ma, info->usb_ma);

	return 0;
}

/* Belows are for using in interrupt context */
static struct chg_cable_info lge_cable_info;

acc_cable_type lge_pm_get_cable_type(void)
{
	return lge_cable_info.cable_type;
}

unsigned lge_pm_get_ta_current(void)
{
	return lge_cable_info.ta_ma;
}

unsigned lge_pm_get_usb_current(void)
{
	return lge_cable_info.usb_ma;
}

/* This must be invoked in process context */
void lge_pm_read_cable_info(void)
{
	lge_cable_info.cable_type = NO_INIT_CABLE;
	lge_cable_info.ta_ma = C_NO_INIT_TA_MA;
	lge_cable_info.usb_ma = C_NO_INIT_USB_MA;

	lge_pm_get_cable_info(&lge_cable_info);
}
#endif /* CONFIG_LGE_PM */
/* END: kidong0420.kim@lge.com 2011-11-09 */

#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
int lge_battery_info = BATT_UNKNOWN;

static int __init battery_information_setup(char *batt_info)
{
	if(!strcmp(batt_info, "ds2704_n"))
		lge_battery_info = BATT_DS2704_N;
	else if(!strcmp(batt_info, "ds2704_l"))
		lge_battery_info = BATT_DS2704_L;
	else if(!strcmp(batt_info, "isl6296_n"))
		lge_battery_info = BATT_ISL6296_N;
	else if(!strcmp(batt_info, "isl6296_l"))
		lge_battery_info = BATT_ISL6296_L;
/* L0, Apply Battery ID Checker after Rev.D
*  Force to return valide value before Rev.D
*  2012-03-19, junsin.park@lge.com
*/
#ifdef CONFIG_MACH_MSM8960_L0
	else if(lge_get_board_revno() < HW_REV_D) {
		batt_info = "ds2704_l";
		lge_battery_info = BATT_DS2704_L;
	}
#endif
	else
		lge_battery_info = BATT_UNKNOWN;

	printk(KERN_INFO "Battery : %s %d\n", batt_info, lge_battery_info);

	return 1;
}
__setup("lge.batt_info=", battery_information_setup);
#endif

/* for supporting two LCDs with one image */
static int maker_id = PRIMARY;

static int __init lcd_maker_id_setup(char *lcd_maker_id)
{
	/* CAUTION : These strings are come from LK. */
	char *maker_str[] = {"primary", "secondary", "reserved",};
	int i;

	for(i=0;i<LCD_MAX;i++)
		if (!strncmp(lcd_maker_id, maker_str[i], 9)) {
			maker_id = (lcd_maker_type) i;
			break;
		}

    printk(KERN_INFO "LCD Maker ID : %s \n", maker_str[maker_id]);
	return 1;
}
__setup("lcd_maker_id=", lcd_maker_id_setup);

#ifdef CONFIG_LGE_KCAL
int g_kcal_r = 255;
int g_kcal_g = 255;
int g_kcal_b = 255;
static int __init display_kcal_setup(char *kcal)
{
	char vaild_k = 0;
	sscanf(kcal, "%d|%d|%d|%c", &g_kcal_r, &g_kcal_g, &g_kcal_b, &vaild_k );
	printk(KERN_INFO "kcal is %d|%d|%d|%c\n",
					g_kcal_r, g_kcal_g, g_kcal_b, vaild_k);

	if(vaild_k != 'K') {
		printk(KERN_INFO "kcal not calibrated yet : %d\n", vaild_k);
		g_kcal_r = g_kcal_g = g_kcal_b = 255;
		printk(KERN_INFO "set to default : %d\n", g_kcal_r);
	}
	return 1;
}
__setup("lge.kcal=", display_kcal_setup);
#endif

/* setting whether uart console is enalbed or disabled */
static int uart_console_mode = 0;

int __init lge_get_uart_mode(void)
{
	return uart_console_mode;
}

static int __init lge_uart_mode(char *uart_mode)
{
	if (!strncmp("enable", uart_mode, 5)) {
		printk(KERN_INFO"UART CONSOLE : enable\n");
		uart_console_mode = 1;
	}
	else
		printk(KERN_INFO"UART CONSOLE : disable\n");

	return 1;
}
__setup("uart_console=", lge_uart_mode);

int lge_get_lcd_maker_id(void)
{
	return maker_id;
}

/* get boot mode information from cmdline.
 * If any boot mode is not specified,
 * boot mode is normal type.
 */
static enum lge_boot_mode_type lge_boot_mode = LGE_BOOT_MODE_NORMAL;
int __init lge_boot_mode_init(char *s)
{
	if (!strcmp(s, "charger"))
		lge_boot_mode = LGE_BOOT_MODE_CHARGER;
	else if (!strcmp(s, "chargerlogo"))
		lge_boot_mode = LGE_BOOT_MODE_CHARGERLOGO;
	else if (!strcmp(s, "factory"))
		lge_boot_mode = LGE_BOOT_MODE_FACTORY;
	else if (!strcmp(s, "factory2"))
		lge_boot_mode = LGE_BOOT_MODE_FACTORY2;
	else if (!strcmp(s, "pifboot"))
		lge_boot_mode = LGE_BOOT_MODE_PIFBOOT;

	return 1;
}
__setup("androidboot.mode=", lge_boot_mode_init);

enum lge_boot_mode_type lge_get_boot_mode(void)
{
	return lge_boot_mode;
}

/* 2011-11-15 taew00k.kang@lge.com 1Seg GSBI10 SPI porting [Start] */
#if defined(CONFIG_LGE_BROADCAST_1SEG) || defined(CONFIG_LGE_BROADCAST_TDMB)
static struct resource resources_qup_spi_gsbi10[] = {
	{
		.name   = "spi_base",
		.start  = MSM_GSBI10_QUP_PHYS,
		.end    = MSM_GSBI10_QUP_PHYS + SZ_4K - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.name   = "gsbi_base",
		.start  = MSM_GSBI10_PHYS,
		.end    = MSM_GSBI10_PHYS + 4 - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.name   = "spi_irq_in",
		.start  = GSBI10_QUP_IRQ,
		.end    = GSBI10_QUP_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_clk",
		.start  = 74,
		.end    = 74,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "spi_cs",
		.start  = 73,
		.end    = 73,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "spi_miso",
		.start  = 72,
		.end    = 72,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "spi_mosi",
		.start  = 71,
		.end    = 71,
		.flags  = IORESOURCE_IO,
	},
};

struct platform_device msm8960_device_qup_spi_gsbi10 = {
	.name	= "spi_qsd",
	.id	= 1,
	.num_resources	= ARRAY_SIZE(resources_qup_spi_gsbi10),
	.resource	= resources_qup_spi_gsbi10,
};
#endif /* CONFIG_LGE_BROADCAST */
/* 2011-11-15 taew00k.kang@lge.com 1Seg GSBI10 SPI porting [End] */


#ifdef CONFIG_ANDROID_RAM_CONSOLE
static struct resource ram_console_resource[] = {
	{
		.name = "ram_console",
		.flags = IORESOURCE_MEM,
	}
};
static struct platform_device ram_console_device = {
	.name = "ram_console",
	.id = -1,
	.num_resources = ARRAY_SIZE(ram_console_resource),
	.resource = ram_console_resource,
};

void __init lge_add_ramconsole_devices(void)
{
	struct resource* res = ram_console_resource;
	struct membank* bank = &meminfo.bank[0];

	res->start = PHYS_OFFSET + bank->size;
	res->end = res->start + LGE_RAM_CONSOLE_SIZE -1;
	printk("RAM CONSOLE START ADDR : 0x%x\n", res->start);
	printk("RAM CONSOLE END ADDR   : 0x%x\n", res->end);

	platform_device_register(&ram_console_device);
}
#endif


#ifdef CONFIG_LGE_HANDLE_PANIC
static struct resource crash_log_resource[] = {
	{
		.name = "crash_log",
		.flags = IORESOURCE_MEM,
	}
};

static struct platform_device panic_handler_device = {
	.name = "panic-handler",
	.num_resources = ARRAY_SIZE(crash_log_resource),
	.resource = crash_log_resource,
	.dev = {
		.platform_data = NULL,
	}
};

volatile resource_size_t lge_add_info;

void __init lge_add_panic_handler_devices(void)
{
	struct resource* res = crash_log_resource;
	struct membank* bank = &meminfo.bank[0];

	res->start = bank->start + bank->size + LGE_RAM_CONSOLE_SIZE;
	res->end = res->start + LGE_CRASH_LOG_SIZE - 1;

	lge_add_info = res->start + LGE_CRASH_LOG_SIZE + (1024 * 4);

	printk(KERN_INFO "CRASH LOG START ADDR : %x\n", res->start);
	printk(KERN_INFO "CRASH LOG END ADDR   : %x\n", res->end);
	printk(KERN_INFO "ADDITIONAL INFO ADDR : %x\n", lge_add_info);

	platform_device_register(&panic_handler_device);
}
#endif

#ifdef CONFIG_LGE_QFPROM_INTERFACE
static struct platform_device qfprom_device = {
	.name = "lge-msm8960-qfprom",
	.id = -1,
};

void __init lge_add_qfprom_devices(void)
{
	platform_device_register(&qfprom_device);
}
#endif
void __init lge_add_gpio_i2c_device(gpio_i2c_init_func_t *init_func)
{
	i2c_init_func[gpio_i2c_dev_num] = init_func;
	gpio_i2c_dev_num++;
}

void __init lge_add_gpio_i2c_devices(void)
{
	int index;
	gpio_i2c_init_func_t *init_func_ptr;

	for (index = 0; index < gpio_i2c_dev_num; index++) {
		init_func_ptr = i2c_init_func[index];
		(*init_func_ptr)(LOWEST_GPIO_I2C_BUS_NUM + index);
	}
}

static void __init register_i2c_devices(void)
{
#ifdef CONFIG_I2C
	u8 mach_mask = 0;
	int i;

	/* Build the matching 'supported_machs' bitmask */
	mach_mask = I2C_SURF;

	/* Run the array and install devices as appropriate */
	for (i = 0; i < msm_i2c_dev_num; i++) {
		if (msm_i2c_devices[i]->machs & mach_mask)
			i2c_register_board_info(msm_i2c_devices[i]->bus,
						msm_i2c_devices[i]->info,
						msm_i2c_devices[i]->len);
	}
#endif
}

void __init lge_add_msm_i2c_device(struct i2c_registry *device)
{
	msm_i2c_devices[msm_i2c_dev_num++] = device;
}

void __init lge_add_i2c_devices(void)
{
	register_i2c_devices();
	lge_add_gpio_i2c_devices();
}
