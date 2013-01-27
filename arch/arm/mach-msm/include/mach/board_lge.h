/* arch/arm/mach-msm/include/mach/board_lge.h
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2011, Code Aurora Forum. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ASM_ARCH_MSM_BOARD_LGE_H
#define __ASM_ARCH_MSM_BOARD_LGE_H

#ifdef CONFIG_I2C
#define MSM_8960_GSBI1_QUP_I2C_BUS_ID 1
#define MSM_8960_GSBI2_QUP_I2C_BUS_ID 2
#define MSM_8960_GSBI3_QUP_I2C_BUS_ID 3
#define MSM_8960_GSBI4_QUP_I2C_BUS_ID 4
#define MSM_8960_GSBI5_QUP_I2C_BUS_ID 5
#define MSM_8960_GSBI6_QUP_I2C_BUS_ID 6
#define MSM_8960_GSBI7_QUP_I2C_BUS_ID 7
#define MSM_8960_GSBI8_QUP_I2C_BUS_ID 8
#define MSM_8960_GSBI9_QUP_I2C_BUS_ID 9
#define MSM_8960_GSBI10_QUP_I2C_BUS_ID 10
#define MSM_8960_GSBI11_QUP_I2C_BUS_ID 11
#define MSM_8960_GSBI12_QUP_I2C_BUS_ID 12

#define I2C_SURF 1
#define I2C_FFA  (1 << 1)
#define I2C_RUMI (1 << 2)
#define I2C_SIM  (1 << 3)
#define I2C_FLUID (1 << 4)

struct i2c_registry {
	u8                     machs;
	int                    bus;
	struct i2c_board_info *info;
	int                    len;
};

typedef void (gpio_i2c_init_func_t)(int bus_num);
#endif

#ifdef CONFIG_ANDROID_RAM_CONSOLE
#define LGE_RAM_CONSOLE_SIZE (124 * SZ_1K * 2)
#endif

#ifdef CONFIG_LGE_HANDLE_PANIC
#define LGE_CRASH_LOG_SIZE (4*SZ_1K)
#endif

typedef enum {
    HW_REV_EVB1 = 0,
    HW_REV_EVB2,
    HW_REV_A,
    HW_REV_B,
    HW_REV_C,
    HW_REV_D,
    HW_REV_E,
    HW_REV_F,
    HW_REV_G,
    HW_REV_H,
    HW_REV_1_0,
    HW_REV_1_1,
    HW_REV_1_2,
    HW_REV_1_3,
    HW_REV_1_4,
    HW_REV_1_5,
    HW_REV_1_6,
    HW_REV_1_7,
    HW_REV_1_8,
    HW_REV_1_9,
    HW_REV_MAX
} hw_rev_type;

hw_rev_type lge_get_board_revno(void);

#ifdef CONFIG_LGE_PM
/* LGE_CHANGE
 * Classified the ADC value for cable detection
 * 2011-12-05, kidong0420.kim@lge.com
 */
typedef enum {
    NO_INIT_CABLE = 0,
    CABLE_MHL_1K,
    CABLE_U_28P7K,
    CABLE_28P7K,
    CABLE_56K,
    CABLE_100K,
    CABLE_130K,
    CABLE_180K,
    CABLE_200K,
    CABLE_220K,
    CABLE_270K,
    CABLE_330K,
    CABLE_620K,
    CABLE_910K,
    CABLE_NONE
} acc_cable_type;

struct chg_cable_info {
	acc_cable_type cable_type;
	unsigned ta_ma;
	unsigned usb_ma;

/* LGE_CHANGE
 * add field for debugging. 
 * 2012-06-21 lee.yonggu@lge.com
 */
	int adc;
	int threshould;
};

int lge_pm_get_cable_info(struct chg_cable_info *);
void lge_pm_read_cable_info(void);
acc_cable_type lge_pm_get_cable_type(void);
unsigned lge_pm_get_ta_current(void);
unsigned lge_pm_get_usb_current(void);
#endif

/* [START] sungsookim */
#ifdef CONFIG_LGE_PM
struct pseudo_batt_info_type {
	int mode;
	int id;
	int therm;
	int temp;
	int volt;
	int capacity;
	int charging;
};
#endif
/* [END] */

#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
enum {
	BATT_DS2704_N = 17,
	BATT_DS2704_L = 32,
	BATT_ISL6296_N = 73,
	BATT_ISL6296_L = 94,
};
extern int lge_battery_info;
#endif

typedef enum {
	PRIMARY = 0,
	SECONDARY,
	LCD_MAX
} lcd_maker_type;

extern int lge_get_lcd_maker_id(void);

#ifdef CONFIG_LGE_KCAL
struct kcal_platform_data {
	int (*set_values)(int r, int g, int b);
	int (*get_values)(int *r, int *g, int *b);
	int (*refresh_display)(void);
};
#endif

/* from androidboot.mode */
enum lge_boot_mode_type {
	LGE_BOOT_MODE_NORMAL = 0,
	LGE_BOOT_MODE_CHARGER,
	LGE_BOOT_MODE_CHARGERLOGO,
	LGE_BOOT_MODE_FACTORY,
	LGE_BOOT_MODE_FACTORY2,
	LGE_BOOT_MODE_PIFBOOT
};

#ifdef CONFIG_SWITCH_FSA8008
struct fsa8008_platform_data {
	const char *switch_name;            /* switch device name */
	const char *keypad_name;			/* keypad device name */

	unsigned int key_code;				/* key code for hook */

	unsigned int gpio_detect;	/* DET : to detect jack inserted or not */
	unsigned int gpio_mic_en;	/* EN : to enable mic */
	unsigned int gpio_jpole;	/* JPOLE : 3pole or 4pole */
	unsigned int gpio_key;		/* S/E button */

	void (*set_headset_mic_bias)(int enable); /* callback function which is initialized while probing */

	unsigned int latency_for_detection; /* latency for pole (3 or 4)detection (in ms) */
};
#endif

#ifdef CONFIG_HALLIC_S5712ACDL1
/* s5712ACDL1 hall ic platform data */
struct s5712ACDL1_platform_data {
	unsigned int irq_pin;
	unsigned int prohibit_time;
};
#endif

/* LGE MHL: Using for MHL Charging When USB or TA cable is connected*/
#ifdef CONFIG_SII8334_MHL_TX
int hdmi_common_cable_connected(void);
int GetMHLConnectedStatus(void);
#endif //CONFIG_SII8334_MHL_TX

#ifdef CONFIG_I2C
void __init lge_add_msm_i2c_device(struct i2c_registry *device);
void __init lge_add_gpio_i2c_device(gpio_i2c_init_func_t *init_func);
#endif

void __init common_device_init(void);
void __init lge_pm_early_init(void);
void __init lge_pm_init(void);
void __init lge_pm_late_init(void);
void __init lge_pm_set_params(void);
void __init lge_add_gpio_i2c_devices(void);
void __init lge_add_i2c_devices(void);
void __init lge_add_input_devices(void);
void __init lge_add_misc_devices(void);
void __init lge_add_sound_devices(void);
void __init lge_add_lcd_devices(void);
void __init lge_add_camera_devices(void);
void __init lge_add_mmc_devices(void);
void __init lge_add_pm_devices(void);
void __init lge_add_usb_devices(void);
#if defined(CONFIG_LGE_NFC_PN544_C2)	
void __init lge_add_nfc_devices(void);
#endif
#ifdef CONFIG_ANDROID_RAM_CONSOLE
void __init lge_add_ramconsole_devices(void);
#endif

#ifdef CONFIG_LGE_HANDLE_PANIC
void __init lge_add_panic_handler_devices(void);
int lge_get_magic_for_subsystem(void);
void lge_set_magic_for_subsystem(const char* subsys_name);
#endif

#ifdef CONFIG_LGE_QFPROM_INTERFACE
void __init lge_add_qfprom_devices(void);
#endif
#ifdef CONFIG_SWITCH_FSA8008
bool lge_get_board_usembhc(void);
#endif

int __init lge_get_uart_mode(void);
enum lge_boot_mode_type lge_get_boot_mode(void);

#ifdef CONFIG_LGE_HIDDEN_RESET
extern int hreset_enable;
extern int on_hidden_reset;

int lge_get_fb_phys_info(unsigned long *start, unsigned long *size);
void *lge_get_hreset_fb_phys_addr(void);
#endif

#endif
