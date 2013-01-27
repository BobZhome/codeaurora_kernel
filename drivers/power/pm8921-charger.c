/* Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
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
#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/mfd/pm8xxx/pm8921-charger.h>
#include <linux/mfd/pm8xxx/pm8921-bms.h>
#include <linux/mfd/pm8xxx/pm8xxx-adc.h>
#include <linux/mfd/pm8xxx/ccadc.h>
#include <linux/mfd/pm8xxx/core.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/workqueue.h>
#include <linux/debugfs.h>
#include <linux/slab.h>

#include <mach/msm_xo.h>
#include <mach/msm_hsusb.h>

#ifdef CONFIG_BATTERY_MAX17043
#include <linux/max17043_fuelgauge.h>
#endif

/* [START] sungsookim */
#if defined (CONFIG_LGE_PM) || defined (CONFIG_LGE_PM_BATTERY_ID_CHECKER)
#include <mach/board_lge.h>
#endif
/* [END] */


#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
#include <linux/reboot.h>
/* LGE_CHANGE
 * Change the charger_temp_scenatio
 * 2012-02-27, jooyeong.lee@lge.com
 */
#include CONFIG_BOARD_HEADER_FILE
#endif

#ifdef CONFIG_LGE_WIRELESS_CHARGER
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/time.h>
#endif

#ifdef CONFIG_MACH_LGE
#include <linux/gpio.h>
#define CONFIG_MACH_LGE_TEMP
#endif

#define CHG_BUCK_CLOCK_CTRL	0x14

#define PBL_ACCESS1		0x04
#define PBL_ACCESS2		0x05
#define SYS_CONFIG_1		0x06
#define SYS_CONFIG_2		0x07
#define CHG_CNTRL		0x204
#define CHG_IBAT_MAX		0x205
#define CHG_TEST		0x206
#define CHG_BUCK_CTRL_TEST1	0x207
#define CHG_BUCK_CTRL_TEST2	0x208
#define CHG_BUCK_CTRL_TEST3	0x209
#define COMPARATOR_OVERRIDE	0x20A
#define PSI_TXRX_SAMPLE_DATA_0	0x20B
#define PSI_TXRX_SAMPLE_DATA_1	0x20C
#define PSI_TXRX_SAMPLE_DATA_2	0x20D
#define PSI_TXRX_SAMPLE_DATA_3	0x20E
#define PSI_CONFIG_STATUS	0x20F
#define CHG_IBAT_SAFE		0x210
#define CHG_ITRICKLE		0x211
#define CHG_CNTRL_2		0x212
#define CHG_VBAT_DET		0x213
#define CHG_VTRICKLE		0x214
#define CHG_ITERM		0x215
#define CHG_CNTRL_3		0x216
#define CHG_VIN_MIN		0x217
#define CHG_TWDOG		0x218
#define CHG_TTRKL_MAX		0x219
#define CHG_TEMP_THRESH		0x21A
#define CHG_TCHG_MAX		0x21B
#define USB_OVP_CONTROL		0x21C
#define DC_OVP_CONTROL		0x21D
#define USB_OVP_TEST		0x21E
#define DC_OVP_TEST		0x21F
#define CHG_VDD_MAX		0x220
#define CHG_VDD_SAFE		0x221
#define CHG_VBAT_BOOT_THRESH	0x222
#define USB_OVP_TRIM		0x355
#define BUCK_CONTROL_TRIM1	0x356
#define BUCK_CONTROL_TRIM2	0x357
#define BUCK_CONTROL_TRIM3	0x358
#define BUCK_CONTROL_TRIM4	0x359
#define CHG_DEFAULTS_TRIM	0x35A
#define CHG_ITRIM		0x35B
#define CHG_TTRIM		0x35C
#define CHG_COMP_OVR		0x20A
#define IUSB_FINE_RES		0x2B6

/* check EOC every 10 seconds */
#define EOC_CHECK_PERIOD_MS	10000
/* check for USB unplug every 200 msecs */
#define UNPLUG_CHECK_WAIT_PERIOD_MS 200

/* LGE_CHANGE_S [dongju99.kim@lge.com] 2012-02-15 */
#ifdef CONFIG_LGE_PM
#define LT_CABLE_56K                6
#define LT_CABLE_130K               7
#define LT_CABLE_910K		    11
#define NO_BATT_QHSUSB_CHG_PORT_DCP 22
#include "../../arch/arm/mach-msm/smd_private.h"
#endif

/* LGE_CHANGE_S [dongju99.kim@lge.com] 2012-07-05 */
#if defined (CONFIG_MACH_MSM8960_VU2SK) || defined (CONFIG_MACH_MSM8960_VU2U) || defined (CONFIG_MACH_MSM8960_VU2KT)
int  rgb_blue_led_control ( void );
extern void lm3533_blue_led_set(int level);
static int blue_led_value = 0 ;
static int lcd_onoff_value = 0;
#endif
/* LGE_CHANGE_E [dongju99.kim@lge.com] 2012-07-05 */ 
enum chg_fsm_state {
	FSM_STATE_OFF_0 = 0,
	FSM_STATE_BATFETDET_START_12 = 12,
	FSM_STATE_BATFETDET_END_16 = 16,
	FSM_STATE_ON_CHG_HIGHI_1 = 1,
	FSM_STATE_ATC_2A = 2,
	FSM_STATE_ATC_2B = 18,
	FSM_STATE_ON_BAT_3 = 3,
	FSM_STATE_ATC_FAIL_4 = 4 ,
	FSM_STATE_DELAY_5 = 5,
	FSM_STATE_ON_CHG_AND_BAT_6 = 6,
	FSM_STATE_FAST_CHG_7 = 7,
	FSM_STATE_TRKL_CHG_8 = 8,
	FSM_STATE_CHG_FAIL_9 = 9,
	FSM_STATE_EOC_10 = 10,
	FSM_STATE_ON_CHG_VREGOK_11 = 11,
	FSM_STATE_ATC_PAUSE_13 = 13,
	FSM_STATE_FAST_CHG_PAUSE_14 = 14,
	FSM_STATE_TRKL_CHG_PAUSE_15 = 15,
	FSM_STATE_START_BOOT = 20,
	FSM_STATE_FLCB_VREGOK = 21,
	FSM_STATE_FLCB = 22,
};

struct fsm_state_to_batt_status {
	enum chg_fsm_state	fsm_state;
	int			batt_state;
};

static struct fsm_state_to_batt_status map[] = {
	{FSM_STATE_OFF_0, POWER_SUPPLY_STATUS_UNKNOWN},
	{FSM_STATE_BATFETDET_START_12, POWER_SUPPLY_STATUS_UNKNOWN},
	{FSM_STATE_BATFETDET_END_16, POWER_SUPPLY_STATUS_UNKNOWN},
	/*
	 * for CHG_HIGHI_1 report NOT_CHARGING if battery missing,
	 * too hot/cold, charger too hot
	 */
	{FSM_STATE_ON_CHG_HIGHI_1, POWER_SUPPLY_STATUS_FULL},
	{FSM_STATE_ATC_2A, POWER_SUPPLY_STATUS_CHARGING},
	{FSM_STATE_ATC_2B, POWER_SUPPLY_STATUS_CHARGING},
	{FSM_STATE_ON_BAT_3, POWER_SUPPLY_STATUS_DISCHARGING},
	{FSM_STATE_ATC_FAIL_4, POWER_SUPPLY_STATUS_DISCHARGING},
	{FSM_STATE_DELAY_5, POWER_SUPPLY_STATUS_UNKNOWN },
	{FSM_STATE_ON_CHG_AND_BAT_6, POWER_SUPPLY_STATUS_CHARGING},
	{FSM_STATE_FAST_CHG_7, POWER_SUPPLY_STATUS_CHARGING},
	{FSM_STATE_TRKL_CHG_8, POWER_SUPPLY_STATUS_CHARGING},
	{FSM_STATE_CHG_FAIL_9, POWER_SUPPLY_STATUS_DISCHARGING},
	{FSM_STATE_EOC_10, POWER_SUPPLY_STATUS_FULL},
	{FSM_STATE_ON_CHG_VREGOK_11, POWER_SUPPLY_STATUS_NOT_CHARGING},
	{FSM_STATE_ATC_PAUSE_13, POWER_SUPPLY_STATUS_NOT_CHARGING},
	{FSM_STATE_FAST_CHG_PAUSE_14, POWER_SUPPLY_STATUS_NOT_CHARGING},
	{FSM_STATE_TRKL_CHG_PAUSE_15, POWER_SUPPLY_STATUS_NOT_CHARGING},
	{FSM_STATE_START_BOOT, POWER_SUPPLY_STATUS_NOT_CHARGING},
	{FSM_STATE_FLCB_VREGOK, POWER_SUPPLY_STATUS_NOT_CHARGING},
	{FSM_STATE_FLCB, POWER_SUPPLY_STATUS_NOT_CHARGING},
};

enum chg_regulation_loop {
	VDD_LOOP = BIT(3),
	BAT_CURRENT_LOOP = BIT(2),
	INPUT_CURRENT_LOOP = BIT(1),
	INPUT_VOLTAGE_LOOP = BIT(0),
	CHG_ALL_LOOPS = VDD_LOOP | BAT_CURRENT_LOOP
			| INPUT_CURRENT_LOOP | INPUT_VOLTAGE_LOOP,
};

enum pmic_chg_interrupts {
	USBIN_VALID_IRQ = 0,
	USBIN_OV_IRQ,
	BATT_INSERTED_IRQ,
	VBATDET_LOW_IRQ,
	USBIN_UV_IRQ,
	VBAT_OV_IRQ,
	CHGWDOG_IRQ,
	VCP_IRQ,
	ATCDONE_IRQ,
	ATCFAIL_IRQ,
	CHGDONE_IRQ,
	CHGFAIL_IRQ,
	CHGSTATE_IRQ,
	LOOP_CHANGE_IRQ,
	FASTCHG_IRQ,
	TRKLCHG_IRQ,
	BATT_REMOVED_IRQ,
	BATTTEMP_HOT_IRQ,
	CHGHOT_IRQ,
	BATTTEMP_COLD_IRQ,
	CHG_GONE_IRQ,
	BAT_TEMP_OK_IRQ,
	COARSE_DET_LOW_IRQ,
	VDD_LOOP_IRQ,
	VREG_OV_IRQ,
	VBATDET_IRQ,
	BATFET_IRQ,
	PSI_IRQ,
	DCIN_VALID_IRQ,
	DCIN_OV_IRQ,
	DCIN_UV_IRQ,
	PM_CHG_MAX_INTS,
};

struct bms_notify {
	int			is_battery_full;
	int			is_charging;
	struct	work_struct	work;
};

/**
 * struct pm8921_chg_chip -device information
 * @dev:			device pointer to access the parent
 * @usb_present:		present status of usb
 * @dc_present:			present status of dc
 * @usb_charger_current:	usb current to charge the battery with used when
 *				the usb path is enabled or charging is resumed
 * @safety_time:		max time for which charging will happen
 * @update_time:		how frequently the userland needs to be updated
 * @max_voltage_mv:		the max volts the batt should be charged up to
 * @min_voltage_mv:		the min battery voltage before turning the FETon
 * @cool_temp_dc:		the cool temp threshold in deciCelcius
 * @warm_temp_dc:		the warm temp threshold in deciCelcius
 * @resume_voltage_delta:	the voltage delta from vdd max at which the
 *				battery should resume charging
 * @term_current:		The charging based term current
 *
 */
struct pm8921_chg_chip {
	struct device			*dev;
	unsigned int			usb_present;
	unsigned int			dc_present;
	unsigned int			usb_charger_current;
	unsigned int			max_bat_chg_current;
	unsigned int			pmic_chg_irq[PM_CHG_MAX_INTS];
	unsigned int			safety_time;
	unsigned int			ttrkl_time;
	unsigned int			update_time;
	unsigned int			max_voltage_mv;
	unsigned int			min_voltage_mv;
	int				cool_temp_dc;
	int				warm_temp_dc;
	unsigned int			temp_check_period;
	unsigned int			cool_bat_chg_current;
	unsigned int			warm_bat_chg_current;
	unsigned int			cool_bat_voltage;
	unsigned int			warm_bat_voltage;
	unsigned int			is_bat_cool;
	unsigned int			is_bat_warm;
	unsigned int			resume_voltage_delta;
	unsigned int			term_current;
	unsigned int			vbat_channel;
	unsigned int			batt_temp_channel;
	unsigned int			batt_id_channel;
#ifdef CONFIG_LGE_WIRELESS_CHARGER
	struct power_supply		wireless_psy;

	struct delayed_work		wireless_chip_control_work;
	struct delayed_work		wireless_unplug_check_work;
	struct delayed_work		wireless_source_control_work;
	struct delayed_work		wireless_polling_dc_work;
	struct work_struct		dcin_valid_irq_work;
	struct work_struct		wireless_interrupt_work;
	struct wake_lock		wireless_unplug_wake_lock;
	struct wake_lock		wireless_chip_wake_lock;
	struct wake_lock		wireless_source_wake_lock;
#endif
	struct delayed_work		wireless_inform_work;

	struct power_supply		usb_psy;
	struct power_supply		dc_psy;
	struct power_supply		*ext_psy;
	struct power_supply		batt_psy;
	struct dentry			*dent;
	struct bms_notify		bms_notify;
	bool				keep_btm_on_suspend;
	bool				ext_charging;
	bool				ext_charge_done;
	bool				iusb_fine_res;
	DECLARE_BITMAP(enabled_irqs, PM_CHG_MAX_INTS);
	struct work_struct		battery_id_valid_work;
	int64_t				batt_id_min;
	int64_t				batt_id_max;
	int				trkl_voltage;
	int				weak_voltage;
	int				trkl_current;
	int				weak_current;
	int				vin_min;
	unsigned int			*thermal_mitigation;
	int				thermal_levels;
	struct delayed_work		update_heartbeat_work;
	struct delayed_work		eoc_work;
#ifdef CONFIG_MACH_LGE_TEMP
	struct work_struct              unplug_ovp_fet_open_work;
#endif
	struct delayed_work		unplug_check_work;
	struct delayed_work		vin_collapse_check_work;
#if defined(CONFIG_LGE_PM) && !defined(CONFIG_BATTERY_MAXIM17043)
	struct wake_lock		batt_alarm_wake_up;
#endif
#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
	struct delayed_work		monitor_batt_temp_work;
	struct wake_lock                monitor_batt_temp_wake_lock;
/* BEGIN : jooyeong.lee@lge.com 2012-02-27 Change the charger_temp_scenario */
	int				temp_level_1;
	int				temp_level_2;
	int				temp_level_3;
	int				temp_level_4;
	int				temp_level_5;
/* END : jooyeong.lee@lge.com 2012-02-27 */
/* LGE_CHANGE
 * add the xo_thermal mitigation way
 * 2012-04-10, hiro.kwon@lge.com
*/
	xo_mitigation_way thermal_mitigation_method;
/* 2012-04-10, hiro.kwon@lge.com */
#endif
	struct wake_lock		eoc_wake_lock;
	enum pm8921_chg_cold_thr	cold_thr;
	enum pm8921_chg_hot_thr		hot_thr;
	int				rconn_mohm;
	enum pm8921_chg_led_src_config	led_src_config;
	int				recent_reported_soc;
#ifdef CONFIG_MACH_LGE
	int batt_id_gpio;		/* No. of msm gpio for battery id */
	int batt_id_pu_gpio;	/* No. of msm gpio for battery id pull up */
#endif
};

/*
#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
enum {
	THERM_M10,
	THERM_M5,
	THERM_42,
	THERM_45,
	THERM_55,
	THERM_57,
	THERM_60,
	THERM_65,
	THERM_LAST,
};

enum {
	DISCHG_BATT_TEMP_OVER_60,
	DISCHG_BATT_TEMP_57_60,
	DISCHG_BATT_TEMP_UNDER_57,
	CHG_BATT_TEMP_OVER_55,
	CHG_BATT_TEMP_46_55,
	CHG_BATT_TEMP_42_45,
	CHG_BATT_TEMP_M4_41,
	CHG_BATT_TEMP_M10_M5,
	CHG_BATT_TEMP_UNDER_M10,
};

enum {
	DISCHG_BATT_NORMAL_STATE,
	DISCHG_BATT_WARNING_STATE,
	DISCHG_BATT_POWEROFF_STATE,
	CHG_BATT_NORMAL_STATE,
	CHG_BATT_DC_CURRENT_STATE,
	CHG_BATT_WARNING_STATE,
	CHG_BATT_STOP_CHARGING_STATE,
};
#endif
*/

#define	WIRELESS_INFORM_RAPID_TIME		5000
#define	WIRELESS_INFORM_NORMAL_TIME		20000

#ifdef CONFIG_LGE_WIRELESS_CHARGER
#undef	WIRELESS_CAYMAN
#ifdef	WIRELESS_CAYMAN
#define	WIRELESS_MAXIM_SOC			95
#define	WIRELESS_BMS_SOC			95
#else
#define	WIRELESS_MAXIM_SOC			99
#define	WIRELESS_BMS_SOC			99
#endif
#define	PM8921_GPIO_BASE			NR_GPIO_IRQS
#define	PM8921_GPIO_PM_TO_SYS(pm_gpio)		(pm_gpio - 1 + PM8921_GPIO_BASE)
#define	CHG_STAT				PM8921_GPIO_PM_TO_SYS(42)
#define	ACTIVE_N				PM8921_GPIO_PM_TO_SYS(25)
#define	WIRELESS_ON				0
#define	WIRELESS_OFF				1


#define	WIRELESS_CHIP_CONTROL_TIME		3000
#define	WIRELESS_EARLY_EOC_WORK_TIME		2500
#define	WIRELESS_EARLY_SOURCE_TIME		1000
//#if defined(CONFIG_MACH_MSM8960_D1LU) || defined(CONFIG_MACH_MSM8960_D1LSK) || defined(CONFIG_MACH_MSM8960_D1LKT) || defined(CONFIG_MACH_MSM8960_L1sk) || defined(CONFIG_MACH_MSM8960_L1kt) || defined(CONFIG_MACH_MSM8960_L1u)
#define	WIRELESS_VIN_MIN			4500
//#elif defined(CONFIG_MACH_MSM8960_D1LV)
//#define	WIRELESS_VIN_MIN			4600
//#else
//#define	WIRELESS_VIN_MIN			4600
//#endif
#define	WIRELESS_IBAT_MAX			525
#define	WIRELESS_DECREASE_IBAT			325
#define	WIRELESS_RESUME_TOLERANCE 		(100)
#define	WIRELESS_ITERM				150
#define WIRELESS_INT_TO_UNPLUG_CHECK		2000
#define	WIRELESS_POLLING_DC_TIME		1000
#define	WIRELESS_UNPLUG_CHECK_TIME		1000
#define	WIRELESS_SRC_COUNT_LIMIT		5
#define WIRELESS_BATFET_BIT			BIT(3)
#define	WIRELESS_VDD_MAX			4350
#define	WIRELESS_VDET_LOW			4350

static	bool	wireless_charging;
static	bool	wireless_charge_done;
static	bool	wireless_source_toggle;
static	int	wireless_source_control_count;
static	int	wireless_inform_no_dc;
//static	bool	wireless_interrupt_enable=true;
//static	long	wireless_dc_time;
//static	long	wireless_on_time;
#ifdef CONFIG_LGE_BACKLIGHT_LM3533
extern	int	wireless_backlight_state(void);
#endif
static	void	wireless_polling_dc_worker(struct work_struct *work);
#ifdef CONFIG_LGE_BACKLIGHT_LM3533
static	bool	wireless_get_backlight_on(void);
#endif
//static	int	wireless_batfet_on(struct pm8921_chg_chip *chip, int enable);
static	int	wireless_pm_power_get_property(struct power_supply *psy);
static	void	wireless_reset_hw(int usb , struct pm8921_chg_chip *chip);
static	void	wireless_set(struct pm8921_chg_chip *chip);
static	void	wireless_reset(struct pm8921_chg_chip *chip);
static	void	wireless_current_set(struct pm8921_chg_chip *chip);
static	void	wireless_vinmin_set(struct pm8921_chg_chip *chip);
static	int	wireless_hw_init(struct pm8921_chg_chip *chip);
static	void	wireless_chip_control_worker(struct work_struct *work);
static	void	wireless_information(struct work_struct *work);
static	int	wireless_batt_status(void);
static	int	wireless_soc(struct pm8921_chg_chip * chip);
static	bool	wireless_soc_check(struct pm8921_chg_chip * chip);
static	bool	wireless_is_plugged(void);
static	void	wireless_unplug_ovp_fet_open(struct pm8921_chg_chip *chip);
static	void	wireless_unplug_check_worker(struct work_struct *work);
static	void	wireless_interrupt_probe(struct pm8921_chg_chip *chip);
static	int	wireless_is_charging_finished(struct pm8921_chg_chip *chip);
static	void	wireless_interrupt_worker(struct work_struct *work);
static	void	wireless_source_control_worker(struct work_struct *work);
static	void	turn_off_dc_ovp_fet(struct pm8921_chg_chip *chip);
static	void	turn_on_dc_ovp_fet(struct pm8921_chg_chip *chip);
static 	void	dcin_valid_irq_worker(struct work_struct *work);
static	irqreturn_t	wireless_interrupt_handler(int irq, void *data);
static	irqreturn_t	dcin_valid_irq_handler(int irq, void *data);
#endif

/* Battery alarm for the model using BMS */
#if defined(CONFIG_LGE_PM) && !defined(CONFIG_BATTERY_MAX17043)
static int msm_battery_gauge_alarm_notify(struct notifier_block *nb,
					  unsigned long status, void *unused);

static struct notifier_block alarm_notifier = {
	.notifier_call = msm_battery_gauge_alarm_notify,
};
#endif


#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
static	bool	pm_is_chg_charge_dis_bit_set(struct pm8921_chg_chip *chip);
static unsigned int last_stop_charging;
static unsigned int chg_batt_temp_state;
/*
 * kiwone.seo@lge.com 2011-0609
 * for show charging ani.although charging is stopped : charging scenario
 */
static int pseudo_ui_charging;
static int stop_charging_status;
/* 120406 mansu.lee@lge.com Implements Thermal Mitigation iUSB setting */
static int pre_xo_mitigation;
/* 120406 mansu.lee@lge.com */

/* LGE_CHANGE
 * fake battery icon display
 * 2012-05-10, hiro.kwon@lge.com
*/
static bool xo_mitigation_stop_chg = false;
static int g_batt_temp = 0;
#endif

/* [START] sungsookim */
#ifdef CONFIG_LGE_PM
static struct pseudo_batt_info_type pseudo_batt_info = {
	.mode = 0,
};

static int block_charging_state = 1; /* 1 : charing, 0 : block charging */
//extern acc_cable_type lge_pm_get_cable_type(void);
#endif
/* [END] */
/* user space parameter to limit usb current */
static unsigned int usb_max_current;
/*
 * usb_target_ma is used for wall charger
 * adaptive input current limiting only. Use
 * pm_iusbmax_get() to get current maximum usb current setting.
 */
static int usb_target_ma;
static int charging_disabled;
static int thermal_mitigation;

static struct pm8921_chg_chip *the_chip;

static struct pm8xxx_adc_arb_btm_param btm_config;
/* 120307 mansu.lee@lge.com Implement Power test SOC quickstart */
#ifdef CONFIG_LGE_PM
int lge_power_test_flag = 0;
#endif
/* 120307 mansu.lee@lge.com */

/* BEGIN : janghyun.baek@lge.com 2012-03-26 for eoc check in charger logo */
#ifdef CONFIG_LGE_PM
static int init_chg_current;
#endif
/* END : janghyun.baek@lge.com 2012-03-26 for eoc check in charger logo */
/* BEGIN : janghyun.baek@lge.com 2012-05-06 for TA/USB recognition */
#ifdef CONFIG_LGE_PM
static int ac_online = 0;
static int usb_online = 0;
#endif
/* END : janghyun.baek@lge.com 2012-05-06 */

/* BEGIN: pk.jeon@lge.com 2012-04-23
 * For Use Battery Health in case BTM Disable in Domestic Operator and DCM (D1LU, D1LSK, D1LKT, L_DCM) */
#if defined(CONFIG_MACH_MSM8960_D1LU) || defined(CONFIG_MACH_MSM8960_D1LSK) || defined(CONFIG_MACH_MSM8960_D1LKT) || defined(CONFIG_MACH_MSM8960_L_DCM) || defined(CONFIG_MACH_MSM8960_L1sk) || defined(CONFIG_MACH_MSM8960_L1kt) || defined(CONFIG_MACH_MSM8960_L1u)
int batt_health = POWER_SUPPLY_HEALTH_GOOD;
#endif
/* END : pk.jeon@lge.com 2012-04-23 */

/* Battery alarm for the model using BMS  */
#if defined(CONFIG_LGE_PM) && !defined(CONFIG_BATTERY_MAX17043)
static int batt_alarm_disabled = 1;
#endif

#ifdef CONFIG_LGE_PM
struct wake_lock uevent_wake_lock;
#endif

/* 120521 mansu.lee@lge.com For touch ic when TA connected. */
#ifdef CONFIG_LGE_PM
int pm8921_charger_is_ta_connected(void)
{
	if (the_chip == NULL)
		return 0;
	return ac_online;
}
EXPORT_SYMBOL(pm8921_charger_is_ta_connected);
#endif
/* 120521 mansu.lee@lge.com */
#ifdef CONFIG_LGE_PM
/*Expose this to entire kernel to control back light for factory cables. kiran.kanneganti@lge.com*/
bool is_factory_cable(void)
{
	unsigned int cable_info;
	unsigned int smem_size;
	acc_cable_type cable_type = lge_pm_get_cable_type();

	/* Add factory cable information stored in SMEM that was updated in sbl3 process.
	 * Because of timing issue, can't get correct cable information using lge_pm_get_cable_type() at probing charger driver.
	 * 2012-04-14, junsin.park@lge.com
	 */
	cable_info = *(unsigned int *)(smem_get_entry(SMEM_ID_VENDOR1, &smem_size));

	if ((cable_type == CABLE_56K ||
		cable_type == CABLE_130K ||
		cable_type == CABLE_910K)||
		(cable_info == LT_CABLE_56K ||
		cable_info == LT_CABLE_130K ||
		cable_info == LT_CABLE_910K))
		return 1;
	else
		return 0;
}
EXPORT_SYMBOL(is_factory_cable);
#endif

#ifdef CONFIG_LGE_PM
void pm8921_charger_force_update_batt_psy(void)
{
	struct pm8921_chg_chip *chip;

	if (!the_chip) {
		pr_err("called before init\n");
		return;
	}

	chip = the_chip;

	power_supply_changed(&chip->batt_psy);
}
EXPORT_SYMBOL_GPL(pm8921_charger_force_update_batt_psy);
#endif

/* Battery alarm for the model using BMS */
#if defined(CONFIG_LGE_PM) && !defined(CONFIG_BATTERY_MAX17043)
int get_bms_battery_uvolts(void)
{
	int rc;
	struct pm8xxx_adc_chan_result result;

	rc = pm8xxx_adc_read(CHANNEL_VBAT, &result);
	if (rc) {
		pr_err("error reading adc channel = %d, rc = %d\n",
					CHANNEL_VBAT, rc);
		return rc;
	}
	pr_debug("mvolts phy = %lld meas = 0x%llx\n", result.physical,
						result.measurement);
	return (int)result.physical;
}

/* Battery alarm for the model using BMS
 * In AP sleep state, SoC can't be updated to user space.
 * To notify user space the information about low battery state, use battery alarm function in PM8921.
 * 2012-06-15, junsin.park@lge.com
 */
#define BATTERY_UPPER_THRESHOLD	4350
#define BATTERY_LOW_THRESHOLD	3600
static int msm_battery_gauge_alarm_notify(struct notifier_block *nb,
		unsigned long status, void *unused)
{
	int soc;
	int voltage;

	wake_lock(&the_chip->batt_alarm_wake_up);
	voltage = get_bms_battery_uvolts()/1000;
	soc = pm8921_bms_get_percent_charge();;

	pr_info("[batt_alarm] origin : voltage = %d / soc = %d\n", voltage, soc);

	if(status == 1){
		pr_info("[batt_alarm] status == 1 \n");
		pr_info("[batt_alarm] batt alarm raised in under low threshold.\n");
	}else if(status == 0){
		pr_info("[batt_alarm] status == 0 \n");
		pr_info("[batt_alarm] into battery voltge ok range. current setting = %d.\n", BATTERY_LOW_THRESHOLD);
	}

	pm8921_charger_force_update_batt_psy();

	wake_unlock(&the_chip->batt_alarm_wake_up);
	return 0;
}
#endif

static int pm_chg_masked_write(struct pm8921_chg_chip *chip, u16 addr,
							u8 mask, u8 val)
{
	int rc;
	u8 reg;

	rc = pm8xxx_readb(chip->dev->parent, addr, &reg);
	if (rc) {
		pr_err("pm8xxx_readb failed: addr=%03X, rc=%d\n", addr, rc);
		return rc;
	}
	reg &= ~mask;
	reg |= val & mask;
	rc = pm8xxx_writeb(chip->dev->parent, addr, reg);
	if (rc) {
		pr_err("pm8xxx_writeb failed: addr=%03X, rc=%d\n", addr, rc);
		return rc;
	}
	return 0;
}

static int pm_chg_get_rt_status(struct pm8921_chg_chip *chip, int irq_id)
{
	return pm8xxx_read_irq_stat(chip->dev->parent,
					chip->pmic_chg_irq[irq_id]);
}

/* Treat OverVoltage/UnderVoltage as source missing */
static int is_usb_chg_plugged_in(struct pm8921_chg_chip *chip)
{
	return pm_chg_get_rt_status(chip, USBIN_VALID_IRQ);
}

/* Treat OverVoltage/UnderVoltage as source missing */
static int is_dc_chg_plugged_in(struct pm8921_chg_chip *chip)
{
	return pm_chg_get_rt_status(chip, DCIN_VALID_IRQ);
}

static int is_batfet_closed(struct pm8921_chg_chip *chip)
{
	return pm_chg_get_rt_status(chip, BATFET_IRQ);
}
#define CAPTURE_FSM_STATE_CMD	0xC2
#define READ_BANK_7		0x70
#define READ_BANK_4		0x40
static int pm_chg_get_fsm_state(struct pm8921_chg_chip *chip)
{
	u8 temp;
	int err, ret = 0;

	temp = CAPTURE_FSM_STATE_CMD;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return err;
	}

	temp = READ_BANK_7;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return err;
	}

	err = pm8xxx_readb(chip->dev->parent, CHG_TEST, &temp);
	if (err) {
		pr_err("pm8xxx_readb fail: addr=%03X, rc=%d\n", CHG_TEST, err);
		return err;
	}
	/* get the lower 4 bits */
	ret = temp & 0xF;

	temp = READ_BANK_4;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return err;
	}

	err = pm8xxx_readb(chip->dev->parent, CHG_TEST, &temp);
	if (err) {
		pr_err("pm8xxx_readb fail: addr=%03X, rc=%d\n", CHG_TEST, err);
		return err;
	}
	/* get the upper 1 bit */
	ret |= (temp & 0x1) << 4;
	return  ret;
}

#define READ_BANK_6		0x60
static int pm_chg_get_regulation_loop(struct pm8921_chg_chip *chip)
{
	u8 temp;
	int err;

	temp = READ_BANK_6;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return err;
	}

	err = pm8xxx_readb(chip->dev->parent, CHG_TEST, &temp);
	if (err) {
		pr_err("pm8xxx_readb fail: addr=%03X, rc=%d\n", CHG_TEST, err);
		return err;
	}

	/* return the lower 4 bits */
	return temp & CHG_ALL_LOOPS;
}

#define CHG_USB_SUSPEND_BIT  BIT(2)
static int pm_chg_usb_suspend_enable(struct pm8921_chg_chip *chip, int enable)
{
	return pm_chg_masked_write(chip, CHG_CNTRL_3, CHG_USB_SUSPEND_BIT,
			enable ? CHG_USB_SUSPEND_BIT : 0);
}

#define CHG_EN_BIT	BIT(7)
static int pm_chg_auto_enable(struct pm8921_chg_chip *chip, int enable)
{
	return pm_chg_masked_write(chip, CHG_CNTRL_3, CHG_EN_BIT,
				enable ? CHG_EN_BIT : 0);
}

#define CHG_FAILED_CLEAR	BIT(0)
#define ATC_FAILED_CLEAR	BIT(1)
static int pm_chg_failed_clear(struct pm8921_chg_chip *chip, int clear)
{
	int rc;

	rc = pm_chg_masked_write(chip, CHG_CNTRL_3, ATC_FAILED_CLEAR,
				clear ? ATC_FAILED_CLEAR : 0);
	rc |= pm_chg_masked_write(chip, CHG_CNTRL_3, CHG_FAILED_CLEAR,
				clear ? CHG_FAILED_CLEAR : 0);
	return rc;
}

#define CHG_CHARGE_DIS_BIT	BIT(1)
static int pm_chg_charge_dis(struct pm8921_chg_chip *chip, int disable)
{
	return pm_chg_masked_write(chip, CHG_CNTRL, CHG_CHARGE_DIS_BIT,
				disable ? CHG_CHARGE_DIS_BIT : 0);
}

static int pm_is_chg_charge_dis(struct pm8921_chg_chip *chip)
{
	u8 temp;

	pm8xxx_readb(chip->dev->parent, CHG_CNTRL, &temp);
	return  temp & CHG_CHARGE_DIS_BIT;
}
#define PM8921_CHG_V_MIN_MV	3240
#define PM8921_CHG_V_STEP_MV	20
#define PM8921_CHG_V_STEP_10MV_OFFSET_BIT	BIT(7)
#define PM8921_CHG_VDDMAX_MAX	4500
#if defined (CONFIG_MACH_MSM8960_L1m)
#define PM8921_CHG_VDDMAX_MIN	3300
#else
#define PM8921_CHG_VDDMAX_MIN	3400
#endif
#define PM8921_CHG_V_MASK	0x7F
static int __pm_chg_vddmax_set(struct pm8921_chg_chip *chip, int voltage)
{
	int remainder;
	u8 temp = 0;

	if (voltage < PM8921_CHG_VDDMAX_MIN
			|| voltage > PM8921_CHG_VDDMAX_MAX) {
		pr_err("bad mV=%d asked to set\n", voltage);
		return -EINVAL;
	}

	temp = (voltage - PM8921_CHG_V_MIN_MV) / PM8921_CHG_V_STEP_MV;

	remainder = voltage % 20;
	if (remainder >= 10) {
		temp |= PM8921_CHG_V_STEP_10MV_OFFSET_BIT;
	}

	pr_debug("voltage=%d setting %02x\n", voltage, temp);
	return pm8xxx_writeb(chip->dev->parent, CHG_VDD_MAX, temp);
}

static int pm_chg_vddmax_get(struct pm8921_chg_chip *chip, int *voltage)
{
	u8 temp;
	int rc;

	rc = pm8xxx_readb(chip->dev->parent, CHG_VDD_MAX, &temp);
	if (rc) {
		pr_err("rc = %d while reading vdd max\n", rc);
		*voltage = 0;
		return rc;
	}
	*voltage = (int)(temp & PM8921_CHG_V_MASK) * PM8921_CHG_V_STEP_MV
							+ PM8921_CHG_V_MIN_MV;
	if (temp & PM8921_CHG_V_STEP_10MV_OFFSET_BIT)
		*voltage =  *voltage + 10;
	return 0;
}

static int pm_chg_vddmax_set(struct pm8921_chg_chip *chip, int voltage)
{
	int current_mv, ret, steps, i;
	bool increase;

	ret = 0;

	if (voltage < PM8921_CHG_VDDMAX_MIN
		|| voltage > PM8921_CHG_VDDMAX_MAX) {
		pr_err("bad mV=%d asked to set\n", voltage);
		return -EINVAL;
	}

	ret = pm_chg_vddmax_get(chip, &current_mv);
	if (ret) {
		pr_err("Failed to read vddmax rc=%d\n", ret);
		return -EINVAL;
	}
	if (current_mv == voltage)
		return 0;

	/* Only change in increments when USB is present */
#ifdef CONFIG_LGE_WIRELESS_CHARGER
	if (is_usb_chg_plugged_in(chip)||wireless_is_plugged()) {
#else
	if (is_usb_chg_plugged_in(chip)) {
#endif
		if (current_mv < voltage) {
			steps = (voltage - current_mv) / PM8921_CHG_V_STEP_MV;
			increase = true;
		} else {
			steps = (current_mv - voltage) / PM8921_CHG_V_STEP_MV;
			increase = false;
		}
		for (i = 0; i < steps; i++) {
			if (increase)
				current_mv += PM8921_CHG_V_STEP_MV;
			else
				current_mv -= PM8921_CHG_V_STEP_MV;
			ret |= __pm_chg_vddmax_set(chip, current_mv);
		}
	}
	ret |= __pm_chg_vddmax_set(chip, voltage);
	return ret;
}
#if defined (CONFIG_MACH_MSM8960_L1m)
#define PM8921_CHG_VDDSAFE_MIN	3300
#else
#define PM8921_CHG_VDDSAFE_MIN	3400
#endif
#define PM8921_CHG_VDDSAFE_MAX	4500
static int pm_chg_vddsafe_set(struct pm8921_chg_chip *chip, int voltage)
{
	u8 temp;

	if (voltage < PM8921_CHG_VDDSAFE_MIN
			|| voltage > PM8921_CHG_VDDSAFE_MAX) {
		pr_err("bad mV=%d asked to set\n", voltage);
		return -EINVAL;
	}
	temp = (voltage - PM8921_CHG_V_MIN_MV) / PM8921_CHG_V_STEP_MV;
	pr_debug("voltage=%d setting %02x\n", voltage, temp);
	return pm_chg_masked_write(chip, CHG_VDD_SAFE, PM8921_CHG_V_MASK, temp);
}

#define PM8921_CHG_VBATDET_MIN	3240
#define PM8921_CHG_VBATDET_MAX	5780
static int pm_chg_vbatdet_set(struct pm8921_chg_chip *chip, int voltage)
{
	u8 temp;

	if (voltage < PM8921_CHG_VBATDET_MIN
			|| voltage > PM8921_CHG_VBATDET_MAX) {
		pr_err("bad mV=%d asked to set\n", voltage);
		return -EINVAL;
	}
	temp = (voltage - PM8921_CHG_V_MIN_MV) / PM8921_CHG_V_STEP_MV;
	pr_debug("voltage=%d setting %02x\n", voltage, temp);
	return pm_chg_masked_write(chip, CHG_VBAT_DET, PM8921_CHG_V_MASK, temp);
}

#define PM8921_CHG_VINMIN_MIN_MV	3800
#define PM8921_CHG_VINMIN_STEP_MV	100
#define PM8921_CHG_VINMIN_USABLE_MAX	6500
#define PM8921_CHG_VINMIN_USABLE_MIN	4300
#define PM8921_CHG_VINMIN_MASK		0x1F
static int pm_chg_vinmin_set(struct pm8921_chg_chip *chip, int voltage)
{
	u8 temp;

	if (voltage < PM8921_CHG_VINMIN_USABLE_MIN
			|| voltage > PM8921_CHG_VINMIN_USABLE_MAX) {
		pr_err("bad mV=%d asked to set\n", voltage);
		return -EINVAL;
	}
	temp = (voltage - PM8921_CHG_VINMIN_MIN_MV) / PM8921_CHG_VINMIN_STEP_MV;
	pr_debug("voltage=%d setting %02x\n", voltage, temp);
	return pm_chg_masked_write(chip, CHG_VIN_MIN, PM8921_CHG_VINMIN_MASK,
									temp);
}

static int pm_chg_vinmin_get(struct pm8921_chg_chip *chip)
{
	u8 temp;
	int rc, voltage_mv;

	rc = pm8xxx_readb(chip->dev->parent, CHG_VIN_MIN, &temp);
	temp &= PM8921_CHG_VINMIN_MASK;

	voltage_mv = PM8921_CHG_VINMIN_MIN_MV +
			(int)temp * PM8921_CHG_VINMIN_STEP_MV;

	return voltage_mv;
}

#define PM8921_CHG_IBATMAX_MID  500
#define PM8921_CHG_IBATMAX_MIN	325
#define PM8921_CHG_IBATMAX_MAX	2000
#define PM8921_CHG_I_MIN_MA	225
#define PM8921_CHG_I_STEP_MA	50
#define PM8921_CHG_I_MASK	0x3F
static int pm_chg_ibatmax_set(struct pm8921_chg_chip *chip, int chg_current)
{
	u8 temp;

	if (chg_current < PM8921_CHG_IBATMAX_MIN
			|| chg_current > PM8921_CHG_IBATMAX_MAX) {
		pr_err("bad mA=%d asked to set\n", chg_current);
		return -EINVAL;
	}
	temp = (chg_current - PM8921_CHG_I_MIN_MA) / PM8921_CHG_I_STEP_MA;
	return pm_chg_masked_write(chip, CHG_IBAT_MAX, PM8921_CHG_I_MASK, temp);
}

#define PM8921_CHG_IBATSAFE_MIN	225
#define PM8921_CHG_IBATSAFE_MAX	3375
static int pm_chg_ibatsafe_set(struct pm8921_chg_chip *chip, int chg_current)
{
	u8 temp;

	if (chg_current < PM8921_CHG_IBATSAFE_MIN
			|| chg_current > PM8921_CHG_IBATSAFE_MAX) {
		pr_err("bad mA=%d asked to set\n", chg_current);
		return -EINVAL;
	}
	temp = (chg_current - PM8921_CHG_I_MIN_MA) / PM8921_CHG_I_STEP_MA;
	return pm_chg_masked_write(chip, CHG_IBAT_SAFE,
						PM8921_CHG_I_MASK, temp);
}

#define PM8921_CHG_ITERM_MIN_MA		50
#define PM8921_CHG_ITERM_MAX_MA		200
#define PM8921_CHG_ITERM_STEP_MA	10
#define PM8921_CHG_ITERM_MASK		0xF
static int pm_chg_iterm_set(struct pm8921_chg_chip *chip, int chg_current)
{
	u8 temp;

	if (chg_current < PM8921_CHG_ITERM_MIN_MA
			|| chg_current > PM8921_CHG_ITERM_MAX_MA) {
		pr_err("bad mA=%d asked to set\n", chg_current);
		return -EINVAL;
	}

	temp = (chg_current - PM8921_CHG_ITERM_MIN_MA)
				/ PM8921_CHG_ITERM_STEP_MA;
	return pm_chg_masked_write(chip, CHG_ITERM, PM8921_CHG_ITERM_MASK,
					 temp);
}

static int pm_chg_iterm_get(struct pm8921_chg_chip *chip, int *chg_current)
{
	u8 temp;
	int rc;

	rc = pm8xxx_readb(chip->dev->parent, CHG_ITERM, &temp);
	if (rc) {
		pr_err("err=%d reading CHG_ITEM\n", rc);
		*chg_current = 0;
		return rc;
	}
	temp &= PM8921_CHG_ITERM_MASK;
	*chg_current = (int)temp * PM8921_CHG_ITERM_STEP_MA
					+ PM8921_CHG_ITERM_MIN_MA;
	return 0;
}

struct usb_ma_limit_entry {
	int	usb_ma;
	u8	value;
};

static struct usb_ma_limit_entry usb_ma_table[] = {
	{100, 0x0},
	{200, 0x1},
	{500, 0x2},
	{600, 0x3},
	{700, 0x4},
	{800, 0x5},
	{850, 0x6},
	{900, 0x8},
	{950, 0x7},
	{1000, 0x9},
	{1100, 0xA},
	{1200, 0xB},
	{1300, 0xC},
	{1400, 0xD},
	{1500, 0xE},
	{1600, 0xF},
};

#define PM8921_CHG_IUSB_MASK 0x1C
#define PM8921_CHG_IUSB_SHIFT 2
#define PM8921_CHG_IUSB_MAX  7
#define PM8921_CHG_IUSB_MIN  0
#define PM8917_IUSB_FINE_RES BIT(0)
static int pm_chg_iusbmax_set(struct pm8921_chg_chip *chip, int reg_val)
{
	u8 temp, fineres;
	int rc;

	fineres = PM8917_IUSB_FINE_RES & usb_ma_table[reg_val].value;
	reg_val = usb_ma_table[reg_val].value >> 1;

	if (reg_val < PM8921_CHG_IUSB_MIN || reg_val > PM8921_CHG_IUSB_MAX) {
		pr_err("bad mA=%d asked to set\n", reg_val);
		return -EINVAL;
	}
	temp = reg_val << PM8921_CHG_IUSB_SHIFT;

	/* IUSB_FINE_RES */
	if (chip->iusb_fine_res) {
		/* Clear IUSB_FINE_RES bit to avoid overshoot */
		rc = pm_chg_masked_write(chip, IUSB_FINE_RES,
			PM8917_IUSB_FINE_RES, 0);

		rc |= pm_chg_masked_write(chip, PBL_ACCESS2,
			PM8921_CHG_IUSB_MASK, temp);

		if (rc) {
			pr_err("Failed to write PBL_ACCESS2 rc=%d\n", rc);
			return rc;
		}

		if (fineres) {
			rc = pm_chg_masked_write(chip, IUSB_FINE_RES,
				PM8917_IUSB_FINE_RES, fineres);
			if (rc)
				pr_err("Failed to write ISUB_FINE_RES rc=%d\n",
					rc);
		}
	} else {
		rc = pm_chg_masked_write(chip, PBL_ACCESS2,
			PM8921_CHG_IUSB_MASK, temp);
		if (rc)
			pr_err("Failed to write PBL_ACCESS2 rc=%d\n", rc);
	}

	return rc;
}

static int pm_chg_iusbmax_get(struct pm8921_chg_chip *chip, int *mA)
{
	u8 temp, fineres;
	int rc, i;

	fineres = 0;
	*mA = 0;
	rc = pm8xxx_readb(chip->dev->parent, PBL_ACCESS2, &temp);
	if (rc) {
		pr_err("err=%d reading PBL_ACCESS2\n", rc);
		return rc;
	}

	if (chip->iusb_fine_res) {
		rc = pm8xxx_readb(chip->dev->parent, IUSB_FINE_RES, &fineres);
		if (rc) {
			pr_err("err=%d reading IUSB_FINE_RES\n", rc);
			return rc;
		}
	}
	temp &= PM8921_CHG_IUSB_MASK;
	temp = temp >> PM8921_CHG_IUSB_SHIFT;

	temp = (temp << 1) | (fineres & PM8917_IUSB_FINE_RES);
	for (i = ARRAY_SIZE(usb_ma_table) - 1; i >= 0; i--) {
		if (usb_ma_table[i].value == temp)
			break;
	}

	*mA = usb_ma_table[i].usb_ma;

	return rc;
}

#define PM8921_CHG_WD_MASK 0x1F
static int pm_chg_disable_wd(struct pm8921_chg_chip *chip)
{
	/* writing 0 to the wd timer disables it */
	return pm_chg_masked_write(chip, CHG_TWDOG, PM8921_CHG_WD_MASK, 0);
}

#define PM8921_CHG_TCHG_MASK	0x7F
#define PM8921_CHG_TCHG_MIN	4
#define PM8921_CHG_TCHG_MAX	512
#define PM8921_CHG_TCHG_STEP	4
static int pm_chg_tchg_max_set(struct pm8921_chg_chip *chip, int minutes)
{
	u8 temp;

	if (minutes < PM8921_CHG_TCHG_MIN || minutes > PM8921_CHG_TCHG_MAX) {
		pr_err("bad max minutes =%d asked to set\n", minutes);
		return -EINVAL;
	}

	temp = (minutes - 1)/PM8921_CHG_TCHG_STEP;
	return pm_chg_masked_write(chip, CHG_TCHG_MAX, PM8921_CHG_TCHG_MASK,
					 temp);
}

#define PM8921_CHG_TTRKL_MASK	0x1F
#define PM8921_CHG_TTRKL_MIN	1
#define PM8921_CHG_TTRKL_MAX	64
static int pm_chg_ttrkl_max_set(struct pm8921_chg_chip *chip, int minutes)
{
	u8 temp;

	if (minutes < PM8921_CHG_TTRKL_MIN || minutes > PM8921_CHG_TTRKL_MAX) {
		pr_err("bad max minutes =%d asked to set\n", minutes);
		return -EINVAL;
	}

	temp = minutes - 1;
	return pm_chg_masked_write(chip, CHG_TTRKL_MAX, PM8921_CHG_TTRKL_MASK,
					 temp);
}

#define PM8921_CHG_VTRKL_MIN_MV		2050
#define PM8921_CHG_VTRKL_MAX_MV		2800
#define PM8921_CHG_VTRKL_STEP_MV	50
#define PM8921_CHG_VTRKL_SHIFT		4
#define PM8921_CHG_VTRKL_MASK		0xF0
static int pm_chg_vtrkl_low_set(struct pm8921_chg_chip *chip, int millivolts)
{
	u8 temp;

	if (millivolts < PM8921_CHG_VTRKL_MIN_MV
			|| millivolts > PM8921_CHG_VTRKL_MAX_MV) {
		pr_err("bad voltage = %dmV asked to set\n", millivolts);
		return -EINVAL;
	}

	temp = (millivolts - PM8921_CHG_VTRKL_MIN_MV)/PM8921_CHG_VTRKL_STEP_MV;
	temp = temp << PM8921_CHG_VTRKL_SHIFT;
	return pm_chg_masked_write(chip, CHG_VTRICKLE, PM8921_CHG_VTRKL_MASK,
					 temp);
}

#define PM8921_CHG_VWEAK_MIN_MV		2100
#define PM8921_CHG_VWEAK_MAX_MV		3600
#define PM8921_CHG_VWEAK_STEP_MV	100
#define PM8921_CHG_VWEAK_MASK		0x0F
static int pm_chg_vweak_set(struct pm8921_chg_chip *chip, int millivolts)
{
	u8 temp;

	if (millivolts < PM8921_CHG_VWEAK_MIN_MV
			|| millivolts > PM8921_CHG_VWEAK_MAX_MV) {
		pr_err("bad voltage = %dmV asked to set\n", millivolts);
		return -EINVAL;
	}

	temp = (millivolts - PM8921_CHG_VWEAK_MIN_MV)/PM8921_CHG_VWEAK_STEP_MV;
	return pm_chg_masked_write(chip, CHG_VTRICKLE, PM8921_CHG_VWEAK_MASK,
					 temp);
}

#define PM8921_CHG_ITRKL_MIN_MA		50
#define PM8921_CHG_ITRKL_MAX_MA		200
#define PM8921_CHG_ITRKL_MASK		0x0F
#define PM8921_CHG_ITRKL_STEP_MA	10
static int pm_chg_itrkl_set(struct pm8921_chg_chip *chip, int milliamps)
{
	u8 temp;

	if (milliamps < PM8921_CHG_ITRKL_MIN_MA
		|| milliamps > PM8921_CHG_ITRKL_MAX_MA) {
		pr_err("bad current = %dmA asked to set\n", milliamps);
		return -EINVAL;
	}

	temp = (milliamps - PM8921_CHG_ITRKL_MIN_MA)/PM8921_CHG_ITRKL_STEP_MA;

	return pm_chg_masked_write(chip, CHG_ITRICKLE, PM8921_CHG_ITRKL_MASK,
					 temp);
}

#define PM8921_CHG_IWEAK_MIN_MA		325
#define PM8921_CHG_IWEAK_MAX_MA		525
#define PM8921_CHG_IWEAK_SHIFT		7
#define PM8921_CHG_IWEAK_MASK		0x80
static int pm_chg_iweak_set(struct pm8921_chg_chip *chip, int milliamps)
{
	u8 temp;

	if (milliamps < PM8921_CHG_IWEAK_MIN_MA
		|| milliamps > PM8921_CHG_IWEAK_MAX_MA) {
		pr_err("bad current = %dmA asked to set\n", milliamps);
		return -EINVAL;
	}

	if (milliamps < PM8921_CHG_IWEAK_MAX_MA)
		temp = 0;
	else
		temp = 1;

	temp = temp << PM8921_CHG_IWEAK_SHIFT;
	return pm_chg_masked_write(chip, CHG_ITRICKLE, PM8921_CHG_IWEAK_MASK,
					 temp);
}

#define PM8921_CHG_BATT_TEMP_THR_COLD	BIT(1)
#define PM8921_CHG_BATT_TEMP_THR_COLD_SHIFT	1
static int pm_chg_batt_cold_temp_config(struct pm8921_chg_chip *chip,
					enum pm8921_chg_cold_thr cold_thr)
{
	u8 temp;

	temp = cold_thr << PM8921_CHG_BATT_TEMP_THR_COLD_SHIFT;
	temp = temp & PM8921_CHG_BATT_TEMP_THR_COLD;
	return pm_chg_masked_write(chip, CHG_CNTRL_2,
					PM8921_CHG_BATT_TEMP_THR_COLD,
					 temp);
}

#define PM8921_CHG_BATT_TEMP_THR_HOT		BIT(0)
#define PM8921_CHG_BATT_TEMP_THR_HOT_SHIFT	0
static int pm_chg_batt_hot_temp_config(struct pm8921_chg_chip *chip,
					enum pm8921_chg_hot_thr hot_thr)
{
	u8 temp;

	temp = hot_thr << PM8921_CHG_BATT_TEMP_THR_HOT_SHIFT;
	temp = temp & PM8921_CHG_BATT_TEMP_THR_HOT;
	return pm_chg_masked_write(chip, CHG_CNTRL_2,
					PM8921_CHG_BATT_TEMP_THR_HOT,
					 temp);
}

#define PM8921_CHG_LED_SRC_CONFIG_SHIFT	4
#define PM8921_CHG_LED_SRC_CONFIG_MASK	0x30
static int pm_chg_led_src_config(struct pm8921_chg_chip *chip,
				enum pm8921_chg_led_src_config led_src_config)
{
	u8 temp;

	if (led_src_config < LED_SRC_GND ||
			led_src_config > LED_SRC_BYPASS)
		return -EINVAL;

	if (led_src_config == LED_SRC_BYPASS)
		return 0;

	temp = led_src_config << PM8921_CHG_LED_SRC_CONFIG_SHIFT;

	return pm_chg_masked_write(chip, CHG_CNTRL_3,
					PM8921_CHG_LED_SRC_CONFIG_MASK, temp);
}

static void disable_input_voltage_regulation(struct pm8921_chg_chip *chip)
{
	u8 temp;
	int rc;

	rc = pm8xxx_writeb(chip->dev->parent, CHG_BUCK_CTRL_TEST3, 0x70);
	if (rc) {
		pr_err("Failed to write 0x70 to CTRL_TEST3 rc = %d\n", rc);
		return;
	}
	rc = pm8xxx_readb(chip->dev->parent, CHG_BUCK_CTRL_TEST3, &temp);
	if (rc) {
		pr_err("Failed to read CTRL_TEST3 rc = %d\n", rc);
		return;
	}
	/* set the input voltage disable bit and the write bit */
	temp |= 0x81;
	rc = pm8xxx_writeb(chip->dev->parent, CHG_BUCK_CTRL_TEST3, temp);
	if (rc) {
		pr_err("Failed to write 0x%x to CTRL_TEST3 rc=%d\n", temp, rc);
		return;
	}
}

static void enable_input_voltage_regulation(struct pm8921_chg_chip *chip)
{
	u8 temp;
	int rc;

	rc = pm8xxx_writeb(chip->dev->parent, CHG_BUCK_CTRL_TEST3, 0x70);
	if (rc) {
		pr_err("Failed to write 0x70 to CTRL_TEST3 rc = %d\n", rc);
		return;
	}
	rc = pm8xxx_readb(chip->dev->parent, CHG_BUCK_CTRL_TEST3, &temp);
	if (rc) {
		pr_err("Failed to read CTRL_TEST3 rc = %d\n", rc);
		return;
	}
	/* unset the input voltage disable bit */
	temp &= 0xFE;
	/* set the write bit */
	temp |= 0x80;
	rc = pm8xxx_writeb(chip->dev->parent, CHG_BUCK_CTRL_TEST3, temp);
	if (rc) {
		pr_err("Failed to write 0x%x to CTRL_TEST3 rc=%d\n", temp, rc);
		return;
	}
}
#ifndef CONFIG_LGE_PM_BATTERY_ID_CHECKER
static int64_t read_battery_id(struct pm8921_chg_chip *chip)
{
	int rc;
	struct pm8xxx_adc_chan_result result;

	rc = pm8xxx_adc_read(chip->batt_id_channel, &result);
	if (rc) {
		pr_err("error reading batt id channel = %d, rc = %d\n",
					chip->vbat_channel, rc);
		return rc;
	}
	pr_debug("batt_id phy = %lld meas = 0x%llx\n", result.physical,
						result.measurement);
	return result.physical;
}
#endif

static int is_battery_valid(struct pm8921_chg_chip *chip)
{
#ifndef CONFIG_LGE_PM_BATTERY_ID_CHECKER
	int64_t rc;
#endif

#ifdef CONFIG_LGE_PM
	if(pseudo_batt_info.mode == 1)
		return 1;
	else if (is_factory_cable())
		return 1;
#endif

#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
	if (lge_battery_info == BATT_DS2704_N || lge_battery_info == BATT_DS2704_L ||
		lge_battery_info == BATT_ISL6296_N || lge_battery_info == BATT_ISL6296_L)
		return 1;
	else
#if defined (CONFIG_MACH_MSM8960_L1m)
	{	
#define THEM_HIGH_VILATAGE 		2000
		int rc;
		struct pm8xxx_adc_chan_result result;

		rc = pm8xxx_adc_read(chip->batt_temp_channel, &result);
		if (rc) {
		pr_err("error reading batt id channel = %d, rc = %d\n",
					chip->batt_temp_channel, rc);
		return rc;
		}
		
		if ((chip->batt_id_min == 0 && chip->batt_id_max == 0) && (result.physical > 0 && result.physical < THEM_HIGH_VILATAGE))
			return 1;
		else
			return 0;
	}
#else
		return 0;
#endif

#else
	if (chip->batt_id_min == 0 && chip->batt_id_max == 0)
		return 1;

	rc = read_battery_id(chip);
	if (rc < 0) {
		pr_err("error reading batt id channel = %d, rc = %lld\n",
					chip->vbat_channel, rc);
		/* assume battery id is valid when adc error happens */
		return 1;
	}

	if (rc < chip->batt_id_min || rc > chip->batt_id_max) {
		pr_err("batt_id phy =%lld is not valid\n", rc);
		return 0;
	}
	return 1;
#endif
}

static void check_battery_valid(struct pm8921_chg_chip *chip)
{
	if (is_battery_valid(chip) == 0) {
		pr_err("batt_id not valid, disbling charging\n");
		pm_chg_auto_enable(chip, 0);
	} else {
		pm_chg_auto_enable(chip, !charging_disabled);
	}
}

static void battery_id_valid(struct work_struct *work)
{
	struct pm8921_chg_chip *chip = container_of(work,
				struct pm8921_chg_chip, battery_id_valid_work);

	check_battery_valid(chip);
}

static void pm8921_chg_enable_irq(struct pm8921_chg_chip *chip, int interrupt)
{
	if (!__test_and_set_bit(interrupt, chip->enabled_irqs)) {
		dev_dbg(chip->dev, "%d\n", chip->pmic_chg_irq[interrupt]);
		enable_irq(chip->pmic_chg_irq[interrupt]);
	}
}

static void pm8921_chg_disable_irq(struct pm8921_chg_chip *chip, int interrupt)
{
	if (__test_and_clear_bit(interrupt, chip->enabled_irqs)) {
		dev_dbg(chip->dev, "%d\n", chip->pmic_chg_irq[interrupt]);
		disable_irq_nosync(chip->pmic_chg_irq[interrupt]);
	}
}

static int pm8921_chg_is_enabled(struct pm8921_chg_chip *chip, int interrupt)
{
	return test_bit(interrupt, chip->enabled_irqs);
}

static bool is_ext_charging(struct pm8921_chg_chip *chip)
{
	union power_supply_propval ret = {0,};

	if (!chip->ext_psy)
		return false;
	if (chip->ext_psy->get_property(chip->ext_psy,
					POWER_SUPPLY_PROP_CHARGE_TYPE, &ret))
		return false;
	if (ret.intval > POWER_SUPPLY_CHARGE_TYPE_NONE)
		return ret.intval;

	return false;
}

static bool is_ext_trickle_charging(struct pm8921_chg_chip *chip)
{
	union power_supply_propval ret = {0,};

	if (!chip->ext_psy)
		return false;
	if (chip->ext_psy->get_property(chip->ext_psy,
					POWER_SUPPLY_PROP_CHARGE_TYPE, &ret))
		return false;
	if (ret.intval == POWER_SUPPLY_CHARGE_TYPE_TRICKLE)
		return true;

	return false;
}

static int is_battery_charging(int fsm_state)
{
	if (is_ext_charging(the_chip))
		return 1;

	switch (fsm_state) {
	case FSM_STATE_ATC_2A:
	case FSM_STATE_ATC_2B:
	case FSM_STATE_ON_CHG_AND_BAT_6:
	case FSM_STATE_FAST_CHG_7:
	case FSM_STATE_TRKL_CHG_8:
		return 1;
	}
	return 0;
}

static void bms_notify(struct work_struct *work)
{
	struct bms_notify *n = container_of(work, struct bms_notify, work);

	if (n->is_charging) {
		pm8921_bms_charging_began();
	} else {
		pm8921_bms_charging_end(n->is_battery_full);
		n->is_battery_full = 0;
	}
}

static void bms_notify_check(struct pm8921_chg_chip *chip)
{
	int fsm_state, new_is_charging;

	fsm_state = pm_chg_get_fsm_state(chip);
	new_is_charging = is_battery_charging(fsm_state);

	if (chip->bms_notify.is_charging ^ new_is_charging) {
		chip->bms_notify.is_charging = new_is_charging;
		schedule_work(&(chip->bms_notify.work));
	}
}

static enum power_supply_property pm_power_props_usb[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static enum power_supply_property pm_power_props_mains[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
};

#ifdef CONFIG_LGE_WIRELESS_CHARGER
static enum power_supply_property pm_power_props_wireless[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
};
#endif

static char *pm_power_supplied_to[] = {
	"battery",
};


#ifdef CONFIG_LGE_WIRELESS_CHARGER
static int pm_power_get_property_wireless(struct power_supply *psy,enum power_supply_property psp,union power_supply_propval *val)
{
	/* Check if called before init */
	if (!the_chip)
		return -EINVAL;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 0;
		if (charging_disabled)
			return 0;

		val->intval = wireless_pm_power_get_property(psy);

		break;
	default:
		return -EINVAL;
	}
	return 0;
}
#endif

#define USB_WALL_THRESHOLD_MA	500
static int pm_power_get_property_mains(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	/* Check if called before init */
	if (!the_chip)
		return -EINVAL;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 0;
		if (charging_disabled)
			return 0;

		/* check external charger first before the dc path */
		if (is_ext_charging(the_chip)) {
			val->intval = 1;
			return 0;
		}

		if (pm_is_chg_charge_dis(the_chip)) {
			val->intval = 0;
			return 0;
		}

#ifndef CONFIG_LGE_WIRELESS_CHARGER
		if (the_chip->dc_present) {
			val->intval = 1;
			return 0;
		}
#endif

/* LGE_S Added by vedhachalam.krishnan@lge.com
	
   code modified to read the ac_online status
   from is_usb_chg_plugged_in function  */

		/* USB with max current greater than 500 mA connected */
		if (usb_target_ma > USB_WALL_THRESHOLD_MA)
			val->intval = is_usb_chg_plugged_in(the_chip);
/* LGE_E */
/* BEGIN : janghyun.baek@lge.com 2012-05-06 for TA/USB recognition */
#ifdef CONFIG_LGE_PM
		if(!is_usb_chg_plugged_in(the_chip))
			val->intval = 0;
		else
			val->intval =ac_online;
#else
		/* USB with max current greater than 500 mA connected */
		if (usb_target_ma > USB_WALL_THRESHOLD_MA)
			val->intval = is_usb_chg_plugged_in(the_chip);
#endif
/* END : janghyun.baek@lge.com 2012-05-06 */

			return 0;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/* BEGIN : janghyun.baek@lge.com 2012-05-06 for TA/USB recognition */
#ifdef CONFIG_LGE_PM
static int pm_power_set_property_mains(struct power_supply *psy,
				  enum power_supply_property psp,
				  const union power_supply_propval *val)
{
	/* Check if called before init */
	if (!the_chip)
		return -EINVAL;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ac_online = val->intval;
		return 1;
	default:
		return -EINVAL;
	}
	return 0;
}
#endif
/* END : janghyun.baek@lge.com 2012-05-06 */

static int pm_power_get_property_usb(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	int current_max;

	/* Check if called before init */
	if (!the_chip)
		return -EINVAL;

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		if (pm_is_chg_charge_dis(the_chip)) {
			val->intval = 0;
		} else {
			pm_chg_iusbmax_get(the_chip, &current_max);
			val->intval = current_max;
		}
		break;
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 0;
		if (charging_disabled)
			return 0;

		/*
		 * if drawing any current from usb is disabled behave
		 * as if no usb cable is connected
		 */
		if (pm_is_chg_charge_dis(the_chip))
			return 0;

		/* USB charging */
/* BEGIN : janghyun.baek@lge.com 2012-05-06 for TA/USB recognition */
#ifdef CONFIG_LGE_PM
		if(!is_usb_chg_plugged_in(the_chip))
			val->intval = 0;
		else
			val->intval = usb_online;
#else
/* LGE_S Added by vedhachalam.krishnan@lge.com
	
   code modified to read the usb_online status
   from is_usb_chg_plugged_in function  */

		if (usb_target_ma < USB_WALL_THRESHOLD_MA)
			val->intval = is_usb_chg_plugged_in(the_chip);
		else
		    return 0;
#endif
/* END : janghyun.baek@lge.com 2012-05-06 */
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
/* BEGIN : janghyun.baek@lge.com 2012-05-06 for TA/USB recognition */
#ifdef CONFIG_LGE_PM
static int pm_power_set_property_usb(struct power_supply *psy,
			    enum power_supply_property psp,
			    const union power_supply_propval *val)
{
	/* Check if called before init */
	if (!the_chip)
		return -EINVAL;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		usb_online = val->intval;
		return 1;
	default:
		return -EINVAL;
	}
	return 0;
}
#endif
/* END : janghyun.baek@lge.com 2012-05-06 */

static enum power_supply_property msm_batt_power_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_ENERGY_FULL,
/* [START] sungsookim */
#ifdef CONFIG_LGE_PM
	POWER_SUPPLY_PROP_PSEUDO_BATT,
	POWER_SUPPLY_PROP_BLOCK_CHARGING,
	POWER_SUPPLY_PROP_EXT_PWR_CHECK,
#endif
/* [END] */
#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
	POWER_SUPPLY_PROP_BATTERY_ID_CHECK,
#endif
/* LGE_CHANGE_S [dongju99.kim@lge.com] 2012-07-05 */ 
#if defined (CONFIG_MACH_MSM8960_VU2SK) || defined (CONFIG_MACH_MSM8960_VU2U) || defined (CONFIG_MACH_MSM8960_VU2KT)
	POWER_SUPPLY_PROP_BLUE,
	POWER_SUPPLY_PROP_LCD_ONOFF,
#endif
/* LGE_CHANGE_E [dongju99.kim@lge.com] 2012-07-05 */ 	
};

static int get_prop_battery_uvolts(struct pm8921_chg_chip *chip)
{
	int rc;
	struct pm8xxx_adc_chan_result result;

	rc = pm8xxx_adc_read(chip->vbat_channel, &result);
	if (rc) {
		pr_err("error reading adc channel = %d, rc = %d\n",
					chip->vbat_channel, rc);
		return rc;
	}
	pr_debug("mvolts phy = %lld meas = 0x%llx\n", result.physical,
						result.measurement);
	return (int)result.physical;
}

#if defined(CONFIG_LGE_CHARGER_TEMP_SCENARIO)

static int get_prop_batt_presence(struct pm8921_chg_chip *chip)
{
#ifdef CONFIG_LGE_PM
	if(pseudo_batt_info.mode == 1)
		return 1;
	else if (is_factory_cable())
		return 1;
#endif

	if (pm_chg_get_rt_status(chip, BATT_INSERTED_IRQ))
		return 1;
	else
		return 0;
}

static int get_prop_batt_present(struct pm8921_chg_chip *chip)
{
#ifdef CONFIG_LGE_PM
	if(pseudo_batt_info.mode == 1)
		return 1;
	else if (is_factory_cable())
		return 1;
#endif

#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
	if ((lge_battery_info == BATT_DS2704_N || lge_battery_info == BATT_DS2704_L ||
		lge_battery_info == BATT_ISL6296_N || lge_battery_info == BATT_ISL6296_L) &&
		pm_chg_get_rt_status(chip, BATT_INSERTED_IRQ))
		return 1;
	else
#if defined (CONFIG_MACH_MSM8960_L1m)
		return 1;
#else
		return 0;
#endif
#else
	return pm_chg_get_rt_status(chip, BATT_INSERTED_IRQ);
#endif
}

#if !defined(CONFIG_BATTERY_MAX17043)
static unsigned int voltage_based_capacity(struct pm8921_chg_chip *chip)
{
	unsigned int current_voltage_uv = get_prop_battery_uvolts(chip);
	unsigned int current_voltage_mv = current_voltage_uv / 1000;
	unsigned int low_voltage = chip->min_voltage_mv;
	unsigned int high_voltage = chip->max_voltage_mv;

	if (current_voltage_mv <= low_voltage)
		return 0;
	else if (current_voltage_mv >= high_voltage)
		return 100;
	else
		return (current_voltage_mv - low_voltage) * 100
		    / (high_voltage - low_voltage);
}

static int get_prop_batt_capacity(struct pm8921_chg_chip *chip)
{
	int percent_soc;

	if (!get_prop_batt_present(chip))
		percent_soc = voltage_based_capacity(chip);
	else
		percent_soc = pm8921_bms_get_percent_charge();

	if (percent_soc == -ENXIO)
		percent_soc = voltage_based_capacity(chip);

	if (percent_soc <= 10)
		pr_warn("low battery charge = %d%%\n", percent_soc);

	chip->recent_reported_soc = percent_soc;
	return percent_soc;
}
#endif
#endif

static int get_prop_batt_current(struct pm8921_chg_chip *chip)
{
	int result_ua, rc;

	rc = pm8921_bms_get_battery_current(&result_ua);
	if (rc == -ENXIO) {
		rc = pm8xxx_ccadc_get_battery_current(&result_ua);
	}

	if (rc) {
		pr_err("unable to get batt current rc = %d\n", rc);
		return rc;
	} else {
		return result_ua;
	}
}

static int get_prop_batt_fcc(struct pm8921_chg_chip *chip)
{
	int rc;

	rc = pm8921_bms_get_fcc();
	if (rc < 0)
		pr_err("unable to get batt fcc rc = %d\n", rc);
	return rc;
}

static int get_prop_batt_health(struct pm8921_chg_chip *chip)
{
/* BEGIN: pk.jeon@lge.com 2012-04-23
* For Use Battery Health in case BTM Disable in Domestic Operator and DCM (D1LU, D1LSK, D1LKT, L_DCM) */
#if defined(CONFIG_MACH_MSM8960_D1LU) || defined(CONFIG_MACH_MSM8960_D1LSK) || defined(CONFIG_MACH_MSM8960_D1LKT) || defined(CONFIG_MACH_MSM8960_L_DCM) || defined(CONFIG_MACH_MSM8960_L1sk) || defined(CONFIG_MACH_MSM8960_L1kt) || defined(CONFIG_MACH_MSM8960_L1u)
		return batt_health;
#else
/* END : pk.jeon@lge.com 2012-04-23 */

	int temp;

	temp = pm_chg_get_rt_status(chip, BATTTEMP_HOT_IRQ);
	if (temp)
		return POWER_SUPPLY_HEALTH_OVERHEAT;

	temp = pm_chg_get_rt_status(chip, BATTTEMP_COLD_IRQ);
	if (temp)
		return POWER_SUPPLY_HEALTH_COLD;

	return POWER_SUPPLY_HEALTH_GOOD;
#endif
}

static int get_prop_charge_type(struct pm8921_chg_chip *chip)
{
	int temp;

	if (!get_prop_batt_present(chip))
		return POWER_SUPPLY_CHARGE_TYPE_NONE;

	if (is_ext_trickle_charging(chip))
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;

	if (is_ext_charging(chip))
		return POWER_SUPPLY_CHARGE_TYPE_FAST;

	temp = pm_chg_get_rt_status(chip, TRKLCHG_IRQ);
	if (temp)
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;

	temp = pm_chg_get_rt_status(chip, FASTCHG_IRQ);
	if (temp)
		return POWER_SUPPLY_CHARGE_TYPE_FAST;

	return POWER_SUPPLY_CHARGE_TYPE_NONE;
}

static int get_prop_batt_status(struct pm8921_chg_chip *chip)
{
	int batt_state = POWER_SUPPLY_STATUS_DISCHARGING;
	int fsm_state = pm_chg_get_fsm_state(chip);
	int i;

/* BEGIN : jooyeong.lee@lge.com 2012-01-19 Change battery status sequence when start charging */
	int batt_capacity;
/* END : jooyeong.lee@lge.com 2012-01-19 */
	if (chip->ext_psy) {
		if (chip->ext_charge_done)
			return POWER_SUPPLY_STATUS_FULL;
		if (chip->ext_charging)
			return POWER_SUPPLY_STATUS_CHARGING;
	}

#ifdef CONFIG_LGE_WIRELESS_CHARGER
	if(wireless_is_plugged()) { //if(is_dc_chg_plugged_in(chip)) {
		batt_state = wireless_batt_status();
		return batt_state;
	}
#endif

#ifdef CONFIG_BATTERY_MAX17043
	batt_capacity = max17043_get_capacity();
#else
	batt_capacity = get_prop_batt_capacity(chip);
#endif

	for (i = 0; i < ARRAY_SIZE(map); i++)
		if (map[i].fsm_state == fsm_state)
			batt_state = map[i].batt_state;
/* BEGIN : jooyeong.lee@lge.com 2012-01-19 Change battery status sequence when start charging */
#ifdef CONFIG_LGE_PM 
	/* Display as a full state when SOC becomes 100%
	 * 2012-06-28, junsin.park@lge.com
	 */
	if((batt_capacity >= 100) && (is_usb_chg_plugged_in(chip)))
		return POWER_SUPPLY_STATUS_FULL;

/* LGE_CHANGE
 * fake battery icon display
 * 2012-05-10, hiro.kwon@lge.com
*/
#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
	if((batt_state == POWER_SUPPLY_STATUS_DISCHARGING ) || (batt_state == POWER_SUPPLY_STATUS_NOT_CHARGING ) || (batt_state == POWER_SUPPLY_STATUS_FULL ))
	{

		if(chip->thermal_mitigation_method == IUSB_USE_FOR_ISYSTEM_METHOD)
		{
			pr_info("xo_mitigation_stop_chg (%d),g_batt_temp (%d)", xo_mitigation_stop_chg, g_batt_temp );

			if(wake_lock_active(&chip->monitor_batt_temp_wake_lock)
			&& (is_usb_chg_plugged_in(chip))
			&&(xo_mitigation_stop_chg == true)
			&&(g_batt_temp < 55)
			&&(g_batt_temp > -10))
			{
				batt_state = POWER_SUPPLY_STATUS_CHARGING;
				return batt_state;
			}

		}
	}
#endif
/* 2012-05-10, hiro.kwon@lge.com */

	if(batt_state == POWER_SUPPLY_STATUS_FULL){
		if( pm_chg_get_rt_status(chip, BATT_INSERTED_IRQ)
			&& is_battery_valid(chip)
			&& pm_chg_get_rt_status(chip, BAT_TEMP_OK_IRQ)
			&& batt_capacity >= 100 ){
			return batt_state;
		}
		/* END : jooyeong.lee@lge.com 2012-01-19*/

#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
	if((chg_batt_temp_state == CHG_BATT_STOP_CHARGING_STATE) && stop_charging_status)
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	if((chg_batt_temp_state == CHG_BATT_STOP_CHARGING_STATE) && !stop_charging_status)
		return POWER_SUPPLY_STATUS_CHARGING;
#endif


/* BEGIN : janghyun.baek@lge.com 2012-05-06 for TA/USB recognition */
#ifdef CONFIG_LGE_PM
		if(!ac_online && !usb_online){
			batt_state = POWER_SUPPLY_STATUS_DISCHARGING;
			return batt_state;
		}
#endif
/* END : janghyun.baek@lge.com 2012-05-06 */

		if( pm_chg_get_rt_status(chip, BATT_INSERTED_IRQ)
			&& is_battery_valid(chip)
			&& pm_chg_get_rt_status(chip, BAT_TEMP_OK_IRQ)
			&& batt_capacity >= 0 && batt_capacity < 100){

			batt_state = POWER_SUPPLY_STATUS_CHARGING;
			return batt_state;
		}
	}
#endif
/* END : jooyeong.lee@lge.com 2012-01-19*/
	if (fsm_state == FSM_STATE_ON_CHG_HIGHI_1) {
#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
		if (!pm_chg_get_rt_status(chip, BATT_INSERTED_IRQ)
			|| !pm_chg_get_rt_status(chip, BAT_TEMP_OK_IRQ)
			|| pm_chg_get_rt_status(chip, CHGHOT_IRQ)
			|| ((pseudo_ui_charging == 0)
			    && (chg_batt_temp_state == CHG_BATT_STOP_CHARGING_STATE)))

			batt_state = POWER_SUPPLY_STATUS_NOT_CHARGING;
#else
		if (!pm_chg_get_rt_status(chip, BATT_INSERTED_IRQ)
			|| !pm_chg_get_rt_status(chip, BAT_TEMP_OK_IRQ)
			|| pm_chg_get_rt_status(chip, CHGHOT_IRQ)
			|| pm_chg_get_rt_status(chip, VBATDET_LOW_IRQ))

			batt_state = POWER_SUPPLY_STATUS_NOT_CHARGING;
#endif
	}
#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
	if((chg_batt_temp_state == CHG_BATT_STOP_CHARGING_STATE)
		&& (is_usb_chg_plugged_in(chip))
		&& stop_charging_status
		&& batt_capacity < 100){

		batt_state = POWER_SUPPLY_STATUS_NOT_CHARGING;
		return batt_state;
	}
#endif
	return batt_state;
}

#define MAX_TOLERABLE_BATT_TEMP_DDC	680
static int get_prop_batt_temp(struct pm8921_chg_chip *chip)
{
	int rc;
	struct pm8xxx_adc_chan_result result;

	rc = pm8xxx_adc_read(chip->batt_temp_channel, &result);
	if (rc) {
		pr_err("error reading adc channel = %d, rc = %d\n",
					chip->vbat_channel, rc);
		return rc;
	}
	pr_debug("batt_temp phy = %lld meas = 0x%llx\n", result.physical,
						result.measurement);
	if (result.physical > MAX_TOLERABLE_BATT_TEMP_DDC)
		pr_err("BATT_TEMP= %d > 68degC, device will be shutdown\n",
							(int) result.physical);

	return (int)result.physical;
}

static int pm_batt_power_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct pm8921_chg_chip *chip = container_of(psy, struct pm8921_chg_chip,
								batt_psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
/* [START] sungsookim */
#ifdef CONFIG_LGE_PM
		if(block_charging_state == 0)
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else
#endif
/* [END] */
		val->intval = get_prop_batt_status(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = get_prop_charge_type(chip);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = get_prop_batt_health(chip);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = get_prop_batt_presence(chip);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = chip->max_voltage_mv * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = chip->min_voltage_mv * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
#ifdef CONFIG_LGE_PM
		if(pseudo_batt_info.mode == 1)
			val->intval = 3990 * 1000;
		/* mansu.lee 2011-01-16 At now, we do not need. */
		/*
		*  else if (is_factory_cable())
		*	val->intval = 3990 * 1000;
		*/
#ifdef CONFIG_BATTERY_MAX17043
		else
			val->intval = max17043_get_voltage() * 1000;
#else
		else
			val->intval = get_prop_battery_uvolts(chip);
#endif
#else
		val->intval = get_prop_battery_uvolts(chip);
#endif
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
#ifdef CONFIG_LGE_PM
		if(pseudo_batt_info.mode == 1)
/* START : dukyong.kim@lge.com 2012-04-07
  * This code is associated with pseudo_batt_mode
  * When setting the pseudo_batt_mode, it works differently depending on models
  */
#ifdef CONFIG_BATTERY_MAX17043
#ifdef CONFIG_MACH_MSM8960_L_DCM
			val->intval = 75;
#else
			val->intval = max17043_get_capacity();
#endif
#else
#ifdef CONFIG_MACH_MSM8960_L1A
			val->intval = 75;

#else
			val->intval = get_prop_batt_capacity(chip);
#endif

#endif
		/* mansu.lee 2011-01-16 At now, we do not need. */
		/*
		*  else if (is_factory_cable())
		*	val->intval = 75;
		*/
/* END : dukyong.kim@lge.com 2012-04-07 */
#ifdef CONFIG_BATTERY_MAX17043
		else
			val->intval = max17043_get_capacity();
#else
		else
			val->intval = get_prop_batt_capacity(chip);
#endif
#else
		val->intval = get_prop_batt_capacity(chip);
#endif
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = get_prop_batt_current(chip);
		break;
/* [START] sungsookim, This code is for testing pseudo_batt_mode, it will be removed soon */
#ifdef CONFIG_LGE_PM
	case POWER_SUPPLY_PROP_TEMP:
		if(pseudo_batt_info.mode == 1)
			val->intval = pseudo_batt_info.temp * 10;
		else if (is_factory_cable())
			val->intval = 25 * 10;
		else
			val->intval = get_prop_batt_temp(chip);
		break;
#else
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = get_prop_batt_temp(chip);
		break;
#endif
/* [END] */
	case POWER_SUPPLY_PROP_ENERGY_FULL:
		val->intval = get_prop_batt_fcc(chip) * 1000;
		break;
/* [START] sungsookim */
#ifdef CONFIG_LGE_PM
	case POWER_SUPPLY_PROP_PSEUDO_BATT:
		val->intval = pseudo_batt_info.mode;
		break;
	case POWER_SUPPLY_PROP_BLOCK_CHARGING:
		val->intval = block_charging_state;
		break;
	case POWER_SUPPLY_PROP_EXT_PWR_CHECK:
		val->intval = lge_pm_get_cable_type();
		break;
#endif
/* [END] */
#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
	case POWER_SUPPLY_PROP_BATTERY_ID_CHECK:
		val->intval = is_battery_valid(chip);
		break;
#endif
/* LGE_CHANGE_S [dongju99.kim@lge.com] 2012-07-05 */ 
#if defined (CONFIG_MACH_MSM8960_VU2SK) || defined (CONFIG_MACH_MSM8960_VU2U) || defined (CONFIG_MACH_MSM8960_VU2KT)
	case POWER_SUPPLY_PROP_BLUE:
		val-> intval = blue_led_value ;// rgb_blue_led_control();
		break;
	case POWER_SUPPLY_PROP_LCD_ONOFF:
		val-> intval = lcd_onoff_value ; //lcd onoff;
		break;		
#endif
/* LGE_CHANGE_E [dongju99.kim@lge.com] 2012-07-05 */ 
	default:
		return -EINVAL;
	}

	return 0;
}


#define CHG_BAT_TEMP_DIS_BIT	BIT(2)
/* [START] sungsookim */
#ifdef CONFIG_LGE_PM
int pseudo_batt_set(struct pseudo_batt_info_type* info)
{
	struct pm8921_chg_chip *chip = the_chip;
	int rc;

	pseudo_batt_info.mode = info->mode;
	pseudo_batt_info.id = info->id;
	pseudo_batt_info.therm = info->therm;
	pseudo_batt_info.temp = info->temp;
	pseudo_batt_info.volt = info->volt;
	pseudo_batt_info.capacity = info->capacity;
	pseudo_batt_info.charging = info->charging;

/* [START] dukyong.kim@lge.com 2012-04-11
 * When Pseudo_batt mode enabled, we let BTM (Battery Temperature Monitoring) disabled.
 * If we do not this, BTM can control itself without our high-temperature operation condition
 *  to prevent permanent damage of the battery.
 * And, this is indicated that customers using external fuel gauge not PM8921 BTM disable the BTM.
 */
	if(pseudo_batt_info.mode)
	{
		rc = pm_chg_masked_write(chip, CHG_CNTRL_2,
		CHG_BAT_TEMP_DIS_BIT, CHG_BAT_TEMP_DIS_BIT);
		if (rc) {
			pr_err("Failed to disable temp control chg rc=%d\n", rc);
		}
	}else	{
		rc = pm_chg_masked_write(chip, CHG_CNTRL_2,
		CHG_BAT_TEMP_DIS_BIT, 0);
		if (rc) {
			pr_err("Failed to enable temp control chg rc=%d\n", rc);
		}
	}
/* [END] dukyong.kim@lge.com 2012-04-11 */

	power_supply_changed(&chip->batt_psy);
	return 0;
}
EXPORT_SYMBOL(pseudo_batt_set);

void block_charging_set(int block)
{
	if(block)
	{
		//pm_chg_auto_disable(0);
		//pm_chg_charge_dis(the_chip, 0);
		pm8921_charger_enable(true);
	}
	else
	{
		//pm_chg_auto_disable(1);
		//pm_chg_auto_enable(the_chip, 1);
		pm8921_charger_enable(false);
	}
}
void batt_block_charging_set(int block)
{
	struct pm8921_chg_chip *chip = the_chip;

	block_charging_state = block;
	block_charging_set(block);

	power_supply_changed(&chip->batt_psy);
}
EXPORT_SYMBOL(batt_block_charging_set);
#endif
/* [END] */
/* LGE_CHANGE_S [dongju99.kim@lge.com] 2012-07-05 */
#if defined (CONFIG_MACH_MSM8960_VU2SK) || defined (CONFIG_MACH_MSM8960_VU2U) || defined (CONFIG_MACH_MSM8960_VU2KT)

#if 0
int rgb_blue_led_control ( void )
{
	int a = 3 ;

//	a = brightness;
	return a ;
}
#endif
void rgb_blue_led_brightness_set(int led )
{
	int brightness = led ;
	blue_led_value = led ;
//	rgb_blue_led_control(brightness);
	lm3533_blue_led_set(brightness); 
return ;	
}
#endif
/* LGE_CHANGE_E [dongju99.kim@lge.com] 2012-07-05 */ 

static void (*notify_vbus_state_func_ptr)(int);
static int usb_chg_current;
static DEFINE_SPINLOCK(vbus_lock);

int pm8921_charger_register_vbus_sn(void (*callback)(int))
{
	pr_debug("%p\n", callback);
	notify_vbus_state_func_ptr = callback;
	return 0;
}
EXPORT_SYMBOL_GPL(pm8921_charger_register_vbus_sn);

/* this is passed to the hsusb via platform_data msm_otg_pdata */
void pm8921_charger_unregister_vbus_sn(void (*callback)(int))
{
	pr_debug("%p\n", callback);
	notify_vbus_state_func_ptr = NULL;
}
EXPORT_SYMBOL_GPL(pm8921_charger_unregister_vbus_sn);

static void notify_usb_of_the_plugin_event(int plugin)
{
	plugin = !!plugin;
	if (notify_vbus_state_func_ptr) {
		pr_debug("notifying plugin\n");
		(*notify_vbus_state_func_ptr) (plugin);
	} else {
		pr_debug("unable to notify plugin\n");
	}
}

/* assumes vbus_lock is held */
static void __pm8921_charger_vbus_draw(unsigned int mA)
{
	int i, rc;
	if (!the_chip) {
		pr_err("called before init\n");
		return;
	}

	if (mA >= 0 && mA <= 2) {
		usb_chg_current = 0;
		rc = pm_chg_iusbmax_set(the_chip, 0);
		printk("dongju99.kim %s usb_chg_current %d\n", __func__, usb_chg_current );
		if (rc) {
			pr_err("unable to set iusb to %d rc = %d\n", 0, rc);
		}
		rc = pm_chg_usb_suspend_enable(the_chip, 1);
		if (rc)
			pr_err("fail to set suspend bit rc=%d\n", rc);
	} else {
		rc = pm_chg_usb_suspend_enable(the_chip, 0);
		if (rc)
			pr_err("fail to reset suspend bit rc=%d\n", rc);
		for (i = ARRAY_SIZE(usb_ma_table) - 1; i >= 0; i--) {
			if (usb_ma_table[i].usb_ma <= mA)
				break;
		}

		/* Check if IUSB_FINE_RES is available */
		if ((usb_ma_table[i].value & PM8917_IUSB_FINE_RES)
				&& !the_chip->iusb_fine_res)
			i--;
		if (i < 0)
			i = 0;
		rc = pm_chg_iusbmax_set(the_chip, i);
		printk("dongju99.kim %s i%d\n", __func__, i );
		if (rc) {
			pr_err("unable to set iusb to %d rc = %d\n", i, rc);
		}
	}
}

/* USB calls these to tell us how much max usb current the system can draw */
void pm8921_charger_vbus_draw(unsigned int mA)
{
	unsigned long flags;

	pr_debug("Enter charge=%d\n", mA);

	if (!the_chip) {
		pr_err("chip not yet initalized\n");
		return;
	}

	/*
	 * Reject VBUS requests if USB connection is the only available
	 * power source. This makes sure that if booting without
	 * battery the iusb_max value is not decreased avoiding potential
	 * brown_outs.
	 *
	 * This would also apply when the battery has been
	 * removed from the running system.
	 */
	if (!get_prop_batt_present(the_chip)
		&& !is_dc_chg_plugged_in(the_chip)) {
		pr_err("rejected: no other power source connected\n");
		return;
	}

	if (usb_max_current && mA > usb_max_current) {
		pr_warn("restricting usb current to %d instead of %d\n",
					usb_max_current, mA);
		mA = usb_max_current;
	}
	if (usb_target_ma == 0 && mA > USB_WALL_THRESHOLD_MA)
		usb_target_ma = mA;

	spin_lock_irqsave(&vbus_lock, flags);
	if (the_chip) {
		if (mA > USB_WALL_THRESHOLD_MA)
			__pm8921_charger_vbus_draw(USB_WALL_THRESHOLD_MA);
		else
			__pm8921_charger_vbus_draw(mA);
	} else {
		/*
		 * called before pmic initialized,
		 * save this value and use it at probe
		 */
		if (mA > USB_WALL_THRESHOLD_MA)
			usb_chg_current = USB_WALL_THRESHOLD_MA;
		else
			usb_chg_current = mA;
	}
	spin_unlock_irqrestore(&vbus_lock, flags);
}
EXPORT_SYMBOL_GPL(pm8921_charger_vbus_draw);

#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
static void pm8921_charger_vbus_draw_internal(unsigned int mA)
{
	unsigned long flags;

	pr_debug("Enter charge=%d\n", mA);
	spin_lock_irqsave(&vbus_lock, flags);
	if (the_chip) {
		__pm8921_charger_vbus_draw(mA);
	} else {
		/*
		 * called before pmic initialized,
		 * save this value and use it at probe
		 */
		usb_chg_current = mA;
	}
	spin_unlock_irqrestore(&vbus_lock, flags);
}
#endif

int pm8921_charger_enable(bool enable)
{
	int rc;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	enable = !!enable;
	rc = pm_chg_auto_enable(the_chip, enable);
	if (rc)
		pr_err("Failed rc=%d\n", rc);
	return rc;
}
EXPORT_SYMBOL(pm8921_charger_enable);

int pm8921_is_usb_chg_plugged_in(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return is_usb_chg_plugged_in(the_chip);
}
EXPORT_SYMBOL(pm8921_is_usb_chg_plugged_in);

int pm8921_is_dc_chg_plugged_in(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return is_dc_chg_plugged_in(the_chip);
}
EXPORT_SYMBOL(pm8921_is_dc_chg_plugged_in);

int pm8921_is_battery_present(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return get_prop_batt_present(the_chip);
}
EXPORT_SYMBOL(pm8921_is_battery_present);

int pm8921_is_batfet_closed(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return is_batfet_closed(the_chip);
}
EXPORT_SYMBOL(pm8921_is_batfet_closed);
/*
 * Disabling the charge current limit causes current
 * current limits to have no monitoring. An adequate charger
 * capable of supplying high current while sustaining VIN_MIN
 * is required if the limiting is disabled.
 */
int pm8921_disable_input_current_limit(bool disable)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	if (disable) {
		pr_warn("Disabling input current limit!\n");

		return pm8xxx_writeb(the_chip->dev->parent,
			 CHG_BUCK_CTRL_TEST3, 0xF2);
	}
	return 0;
}
EXPORT_SYMBOL(pm8921_disable_input_current_limit);

int pm8921_set_max_battery_charge_current(int ma)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return pm_chg_ibatmax_set(the_chip, ma);
}
EXPORT_SYMBOL(pm8921_set_max_battery_charge_current);

int pm8921_disable_source_current(bool disable)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	if (disable)
		printk("current drawn from chg=0, battery provides current\n");
	return pm_chg_charge_dis(the_chip, disable);
}
EXPORT_SYMBOL(pm8921_disable_source_current);

int pm8921_regulate_input_voltage(int voltage)
{
	int rc;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	rc = pm_chg_vinmin_set(the_chip, voltage);

	if (rc == 0)
		the_chip->vin_min = voltage;

	return rc;
}

#define USB_OV_THRESHOLD_MASK  0x60
#define USB_OV_THRESHOLD_SHIFT  5
int pm8921_usb_ovp_set_threshold(enum pm8921_usb_ov_threshold ov)
{
	u8 temp;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (ov > PM_USB_OV_7V) {
		pr_err("limiting to over voltage threshold to 7volts\n");
		ov = PM_USB_OV_7V;
	}

	temp = USB_OV_THRESHOLD_MASK & (ov << USB_OV_THRESHOLD_SHIFT);

	return pm_chg_masked_write(the_chip, USB_OVP_CONTROL,
				USB_OV_THRESHOLD_MASK, temp);
}
EXPORT_SYMBOL(pm8921_usb_ovp_set_threshold);

#define USB_DEBOUNCE_TIME_MASK	0x06
#define USB_DEBOUNCE_TIME_SHIFT 1
int pm8921_usb_ovp_set_hystersis(enum pm8921_usb_debounce_time ms)
{
	u8 temp;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (ms > PM_USB_DEBOUNCE_80P5MS) {
		pr_err("limiting debounce to 80.5ms\n");
		ms = PM_USB_DEBOUNCE_80P5MS;
	}

	temp = USB_DEBOUNCE_TIME_MASK & (ms << USB_DEBOUNCE_TIME_SHIFT);

	return pm_chg_masked_write(the_chip, USB_OVP_CONTROL,
				USB_DEBOUNCE_TIME_MASK, temp);
}
EXPORT_SYMBOL(pm8921_usb_ovp_set_hystersis);

#define USB_OVP_DISABLE_MASK	0x80
int pm8921_usb_ovp_disable(int disable)
{
	u8 temp = 0;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (disable)
		temp = USB_OVP_DISABLE_MASK;

	return pm_chg_masked_write(the_chip, USB_OVP_CONTROL,
				USB_OVP_DISABLE_MASK, temp);
}

bool pm8921_is_battery_charging(int *source)
{
	int fsm_state, is_charging, dc_present, usb_present;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	fsm_state = pm_chg_get_fsm_state(the_chip);
	is_charging = is_battery_charging(fsm_state);
	if (is_charging == 0) {
		*source = PM8921_CHG_SRC_NONE;
		return is_charging;
	}

	if (source == NULL)
		return is_charging;

	/* the battery is charging, the source is requested, find it */
	dc_present = is_dc_chg_plugged_in(the_chip);
	usb_present = is_usb_chg_plugged_in(the_chip);

	if (dc_present && !usb_present)
		*source = PM8921_CHG_SRC_DC;

	if (usb_present && !dc_present)
		*source = PM8921_CHG_SRC_USB;

	if (usb_present && dc_present)
		/*
		 * The system always chooses dc for charging since it has
		 * higher priority.
		 */
		*source = PM8921_CHG_SRC_DC;

	return is_charging;
}
EXPORT_SYMBOL(pm8921_is_battery_charging);

int pm8921_set_usb_power_supply_type(enum power_supply_type type)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (type < POWER_SUPPLY_TYPE_USB)
		return -EINVAL;

	the_chip->usb_psy.type = type;
	power_supply_changed(&the_chip->usb_psy);
	power_supply_changed(&the_chip->dc_psy);
	return 0;
}
EXPORT_SYMBOL_GPL(pm8921_set_usb_power_supply_type);

int pm8921_batt_temperature(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return get_prop_batt_temp(the_chip);
}

static void handle_usb_insertion_removal(struct pm8921_chg_chip *chip)
{
	int usb_present;

#ifdef CONFIG_LGE_PM
	if(uevent_wake_lock.name != NULL)
	wake_lock_timeout(&uevent_wake_lock, HZ*2);
#endif

	pm_chg_failed_clear(chip, 1);
	usb_present = is_usb_chg_plugged_in(chip);
	if (chip->usb_present ^ usb_present) {
		notify_usb_of_the_plugin_event(usb_present);
		chip->usb_present = usb_present;
		power_supply_changed(&chip->usb_psy);
		power_supply_changed(&chip->batt_psy);
#ifdef CONFIG_LGE_PM
		power_supply_changed(&chip->dc_psy);
#endif
		pm8921_bms_calibrate_hkadc();
	}
#ifdef CONFIG_LGE_WIRELESS_CHARGER
	wireless_reset_hw(usb_present,chip);
#else
	/* LGE_CHANGE
   	 * add the xo_thermal mitigation way
 	 * 2012-04-20, hiro.kwon@lge.com
 	*/

	 if(chip->thermal_mitigation_method == IUSB_USE_FOR_ISYSTEM_METHOD)
	{
		pm_chg_auto_enable(chip,1);
		if(delayed_work_pending(&chip->monitor_batt_temp_work))
		{
			cancel_delayed_work(&chip->monitor_batt_temp_work);
			schedule_delayed_work(&chip->monitor_batt_temp_work,round_jiffies_relative(msecs_to_jiffies(5000)));
		}
	}
	/* 2012-04-20, hiro.kwon@lge.com */
#endif
#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
	if (usb_present) {
	    /*
	     * In monitor_batt_temp, last_stop_charging is exclusive with pm_chg_auto_enable().
	     * Before only in IUSB_REDUCE_METHOD, pm_chg_auto_enable() could be 0, although last_stop_charging was 0.
	     * If this happened, the device could not be recharged before powercycling.
	     * If last_stop_charging is reset as '0', pm_chg_auto_enable() should be set as '1'.
		 * 2012.10.24 lee.yonggu@lge.com
	     */
        if(chip->thermal_mitigation_method == IUSB_REDUCE_METHOD)
		{
			pm_chg_auto_enable(chip,1);

			schedule_delayed_work(&chip->monitor_batt_temp_work,round_jiffies_relative(msecs_to_jiffies(5000)));
		}

		last_stop_charging = 0;
		pseudo_ui_charging = 0;
		/* 120406 mansu.lee@lge.com Implements Thermal Mitigation iUSB setting */
		pre_xo_mitigation = 0;
		/* 120406 mansu.lee@lge.com */
	}
#endif
	if (usb_present) {
		schedule_delayed_work(&chip->unplug_check_work,
			round_jiffies_relative(msecs_to_jiffies
				(UNPLUG_CHECK_WAIT_PERIOD_MS)));
		pm8921_chg_enable_irq(chip, CHG_GONE_IRQ);
	} else {
		/* USB unplugged reset target current */
		usb_target_ma = 0;
		pm8921_chg_disable_irq(chip, CHG_GONE_IRQ);
	}
	enable_input_voltage_regulation(chip);
	bms_notify_check(chip);
}

static void handle_stop_ext_chg(struct pm8921_chg_chip *chip)
{
	if (!chip->ext_psy) {
		pr_debug("external charger not registered.\n");
		return;
	}

	if (!chip->ext_charging) {
		pr_debug("already not charging.\n");
		return;
	}

	power_supply_set_charge_type(chip->ext_psy,
					POWER_SUPPLY_CHARGE_TYPE_NONE);
	pm8921_disable_source_current(false); /* release BATFET */
	power_supply_changed(&chip->dc_psy);
	chip->ext_charging = false;
	chip->ext_charge_done = false;
	bms_notify_check(chip);
	/* Update battery charging LEDs and user space battery info */
	power_supply_changed(&chip->batt_psy);
}

static void handle_start_ext_chg(struct pm8921_chg_chip *chip)
{
	int dc_present;
	int batt_present;
	int batt_temp_ok;
	int vbat_ov;
	unsigned long delay =
		round_jiffies_relative(msecs_to_jiffies(EOC_CHECK_PERIOD_MS));

	if (!chip->ext_psy) {
		pr_debug("external charger not registered.\n");
		return;
	}

	if (chip->ext_charging) {
		pr_debug("already charging.\n");
		return;
	}

	dc_present = is_dc_chg_plugged_in(the_chip);
	batt_present = pm_chg_get_rt_status(chip, BATT_INSERTED_IRQ);
	batt_temp_ok = pm_chg_get_rt_status(chip, BAT_TEMP_OK_IRQ);

	if (!dc_present) {
		pr_warn("%s. dc not present.\n", __func__);
		return;
	}
	if (!batt_present) {
		pr_warn("%s. battery not present.\n", __func__);
		return;
	}
	if (!batt_temp_ok) {
		pr_warn("%s. battery temperature not ok.\n", __func__);
		return;
	}
	pm8921_disable_source_current(true); /* Force BATFET=ON */
	vbat_ov = pm_chg_get_rt_status(chip, VBAT_OV_IRQ);
	if (vbat_ov) {
		pr_warn("%s. battery over voltage.\n", __func__);
		return;
	}

	power_supply_set_online(chip->ext_psy, dc_present);
	power_supply_set_charge_type(chip->ext_psy,
					POWER_SUPPLY_CHARGE_TYPE_FAST);
	power_supply_changed(&chip->dc_psy);
	chip->ext_charging = true;
	chip->ext_charge_done = false;
	bms_notify_check(chip);
	/* Start BMS */
	schedule_delayed_work(&chip->eoc_work, delay);
	wake_lock(&chip->eoc_wake_lock);
	/* Update battery charging LEDs and user space battery info */
	power_supply_changed(&chip->batt_psy);
}

static void turn_off_usb_ovp_fet(struct pm8921_chg_chip *chip)
{
	u8 temp;
	int rc;

	rc = pm8xxx_writeb(chip->dev->parent, USB_OVP_TEST, 0x30);
	if (rc) {
		pr_err("Failed to write 0x30 to USB_OVP_TEST rc = %d\n", rc);
		return;
	}
	rc = pm8xxx_readb(chip->dev->parent, USB_OVP_TEST, &temp);
	if (rc) {
		pr_err("Failed to read from USB_OVP_TEST rc = %d\n", rc);
		return;
	}
	/* set ovp fet disable bit and the write bit */
	temp |= 0x81;
	rc = pm8xxx_writeb(chip->dev->parent, USB_OVP_TEST, temp);
	if (rc) {
		pr_err("Failed to write 0x%x USB_OVP_TEST rc=%d\n", temp, rc);
		return;
	}
}

static void turn_on_usb_ovp_fet(struct pm8921_chg_chip *chip)
{
	u8 temp;
	int rc;

	rc = pm8xxx_writeb(chip->dev->parent, USB_OVP_TEST, 0x30);
	if (rc) {
		pr_err("Failed to write 0x30 to USB_OVP_TEST rc = %d\n", rc);
		return;
	}
	rc = pm8xxx_readb(chip->dev->parent, USB_OVP_TEST, &temp);
	if (rc) {
		pr_err("Failed to read from USB_OVP_TEST rc = %d\n", rc);
		return;
	}
	/* unset ovp fet disable bit and set the write bit */
	temp &= 0xFE;
	temp |= 0x80;
	rc = pm8xxx_writeb(chip->dev->parent, USB_OVP_TEST, temp);
	if (rc) {
		pr_err("Failed to write 0x%x to USB_OVP_TEST rc = %d\n",
								temp, rc);
		return;
	}
}

#if defined (CONFIG_MACH_MSM8960_L1m)
static int param_open_ovp_counter = 15;
#else
static int param_open_ovp_counter = 10;
#endif
module_param(param_open_ovp_counter, int, 0644);

#define WRITE_BANK_4		0xC0
#define USB_OVP_DEBOUNCE_TIME 0x06
#ifdef CONFIG_MACH_LGE_TEMP
static void unplug_ovp_fet_open_worker(struct work_struct *work)
#else
static void unplug_ovp_fet_open(struct pm8921_chg_chip *chip)
#endif
{

#ifdef CONFIG_MACH_LGE_TEMP
       struct pm8921_chg_chip *chip = container_of(work,
                               struct pm8921_chg_chip,
                               unplug_ovp_fet_open_work);
#endif

	int chg_gone, usb_chg_plugged_in;
	int count = 0;

	while (count++ < param_open_ovp_counter) {
		pm_chg_masked_write(chip, USB_OVP_CONTROL,
						USB_OVP_DEBOUNCE_TIME, 0x0);
		usleep(10);
		usb_chg_plugged_in = is_usb_chg_plugged_in(chip);
		chg_gone = pm_chg_get_rt_status(chip, CHG_GONE_IRQ);
		pr_debug("OVP FET count = %d chg_gone=%d, usb_valid = %d\n",
					count, chg_gone, usb_chg_plugged_in);

		/* note usb_chg_plugged_in=0 => chg_gone=1 */
		if (chg_gone == 1 && usb_chg_plugged_in == 1) {
			pr_debug("since chg_gone = 1 dis ovp_fet for 20msec\n");
			turn_off_usb_ovp_fet(chip);

			msleep(20);

			turn_on_usb_ovp_fet(chip);
		} else {
			break;
		}
	}
	pm_chg_masked_write(chip, USB_OVP_CONTROL,
		USB_OVP_DEBOUNCE_TIME, 0x2);
	pr_debug("Exit count=%d chg_gone=%d, usb_valid=%d\n",
		count, chg_gone, usb_chg_plugged_in);
	return;
}

static int find_usb_ma_value(int value)
{
	int i;

	for (i = ARRAY_SIZE(usb_ma_table) - 1; i >= 0; i--) {
		if (value >= usb_ma_table[i].usb_ma)
			break;
	}

	return i;
}

static void decrease_usb_ma_value(int *value)
{
	int i;

	if (value) {
		i = find_usb_ma_value(*value);
		if (i > 0)
			i--;
		*value = usb_ma_table[i].usb_ma;
	}
}

static void increase_usb_ma_value(int *value)
{
	int i;

	if (value) {
		i = find_usb_ma_value(*value);

		if (i < (ARRAY_SIZE(usb_ma_table) - 1))
			i++;
		/* Get next correct entry if IUSB_FINE_RES is not available */
		while (!the_chip->iusb_fine_res
			&& (usb_ma_table[i].value & PM8917_IUSB_FINE_RES)
			&& i < (ARRAY_SIZE(usb_ma_table) - 1))
			i++;

		*value = usb_ma_table[i].usb_ma;
	}
}

static void vin_collapse_check_worker(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct pm8921_chg_chip *chip = container_of(dwork,
			struct pm8921_chg_chip, vin_collapse_check_work);

	/* AICL only for wall-chargers */
	if (is_usb_chg_plugged_in(chip) &&
		usb_target_ma > USB_WALL_THRESHOLD_MA) {
		/* decrease usb_target_ma */
		decrease_usb_ma_value(&usb_target_ma);
		/* reset here, increase in unplug_check_worker */
		__pm8921_charger_vbus_draw(USB_WALL_THRESHOLD_MA);
		pr_debug("usb_now=%d, usb_target = %d\n",
				USB_WALL_THRESHOLD_MA, usb_target_ma);
	} else {
		handle_usb_insertion_removal(chip);
	}
}

#define VIN_MIN_COLLAPSE_CHECK_MS	50

static irqreturn_t usbin_valid_irq_handler(int irq, void *data)
{
	if (usb_target_ma)
		schedule_delayed_work(&the_chip->vin_collapse_check_work,
				      round_jiffies_relative(msecs_to_jiffies
						(VIN_MIN_COLLAPSE_CHECK_MS)));
	else
	    handle_usb_insertion_removal(data);
	return IRQ_HANDLED;
}

static irqreturn_t usbin_ov_irq_handler(int irq, void *data)
{
	pr_err("USB OverVoltage\n");
	return IRQ_HANDLED;
}

static irqreturn_t batt_inserted_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;
	int status;

#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
#ifdef CONFIG_LGE_WIRELESS_CHARGER
	if((is_usb_chg_plugged_in(chip) || wireless_is_plugged())
#else
	if(is_usb_chg_plugged_in(chip)
#endif
		&& !is_factory_cable() && !pseudo_batt_info.mode){
		pr_info("[PM] Battery inserted but cable exists. Now reset as scenario !!\n");
		kernel_restart("battery");
	}
#endif

	status = pm_chg_get_rt_status(chip, BATT_INSERTED_IRQ);
	schedule_work(&chip->battery_id_valid_work);
	handle_start_ext_chg(chip);
	pr_debug("battery present=%d", status);
	power_supply_changed(&chip->batt_psy);
	return IRQ_HANDLED;
}

/*
 * this interrupt used to restart charging a battery.
 *
 * Note: When DC-inserted the VBAT can't go low.
 * VPH_PWR is provided by the ext-charger.
 * After End-Of-Charging from DC, charging can be resumed only
 * if DC is removed and then inserted after the battery was in use.
 * Therefore the handle_start_ext_chg() is not called.
 */
static irqreturn_t vbatdet_low_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;
	int high_transition;

	high_transition = pm_chg_get_rt_status(chip, VBATDET_LOW_IRQ);
#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
	if((chg_batt_temp_state != CHG_BATT_STOP_CHARGING_STATE) && (lge_power_test_flag != 1)){
#endif
		if (high_transition) {
			/* enable auto charging */
			pm_chg_auto_enable(chip, !charging_disabled);
			pr_info("batt fell below resume voltage %s\n",
				charging_disabled ? "" : "charger enabled");
		}
#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
	}
	else
		pr_debug("batt fell below resume voltage or POWER TEST: charger disabled\n");
#endif
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));

	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->usb_psy);
	power_supply_changed(&chip->dc_psy);

	return IRQ_HANDLED;
}

static irqreturn_t usbin_uv_irq_handler(int irq, void *data)
{
	pr_err("USB UnderVoltage\n");
	return IRQ_HANDLED;
}

static irqreturn_t vbat_ov_irq_handler(int irq, void *data)
{
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t chgwdog_irq_handler(int irq, void *data)
{
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t vcp_irq_handler(int irq, void *data)
{
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t atcdone_irq_handler(int irq, void *data)
{
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t atcfail_irq_handler(int irq, void *data)
{
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t chgdone_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	pr_debug("state_changed_to=%d\n", pm_chg_get_fsm_state(data));

	handle_stop_ext_chg(chip);

	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->usb_psy);
	power_supply_changed(&chip->dc_psy);

	bms_notify_check(chip);

	return IRQ_HANDLED;
}

static irqreturn_t chgfail_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;
	int ret;

	ret = pm_chg_failed_clear(chip, 1);
	if (ret)
		pr_err("Failed to write CHG_FAILED_CLEAR bit\n");

	pr_err("batt_present = %d, batt_temp_ok = %d, state_changed_to=%d\n",
			get_prop_batt_present(chip),
			pm_chg_get_rt_status(chip, BAT_TEMP_OK_IRQ),
			pm_chg_get_fsm_state(data));

	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->usb_psy);
	power_supply_changed(&chip->dc_psy);
	return IRQ_HANDLED;
}

static irqreturn_t chgstate_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;
#ifdef CONFIG_LGE_WIRELESS_CHARGER
	printk(KERN_INFO "[WIRELESS] CHGSTATE FSM=%d\n", pm_chg_get_fsm_state(chip));
#else
	pr_debug("state_changed_to=%d\n", pm_chg_get_fsm_state(data));
	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->usb_psy);
	power_supply_changed(&chip->dc_psy);

	bms_notify_check(chip);
#endif
	return IRQ_HANDLED;
}

static int param_vin_disable_counter = 5;
module_param(param_vin_disable_counter, int, 0644);

static void attempt_reverse_boost_fix(struct pm8921_chg_chip *chip,
							int count, int usb_ma)
{
	__pm8921_charger_vbus_draw(500);
	pr_debug("count = %d iusb=500mA\n", count);
	disable_input_voltage_regulation(chip);
	pr_debug("count = %d disable_input_regulation\n", count);

	msleep(20);

	pr_debug("count = %d end sleep 20ms chg_gone=%d, usb_valid = %d\n",
				count,
				pm_chg_get_rt_status(chip, CHG_GONE_IRQ),
				is_usb_chg_plugged_in(chip));
	pr_debug("count = %d restoring input regulation and usb_ma = %d\n",
		 count, usb_ma);
	enable_input_voltage_regulation(chip);
	__pm8921_charger_vbus_draw(usb_ma);
}

#define VIN_ACTIVE_BIT BIT(0)
#define UNPLUG_WRKARND_RESTORE_WAIT_PERIOD_US 200
#define VIN_MIN_INCREASE_MV 100
static void unplug_check_worker(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct pm8921_chg_chip *chip = container_of(dwork,
				struct pm8921_chg_chip, unplug_check_work);
	u8 reg_loop;
	int ibat, usb_chg_plugged_in, usb_ma;
	int chg_gone = 0;

	reg_loop = 0;
	usb_chg_plugged_in = is_usb_chg_plugged_in(chip);
	if (!usb_chg_plugged_in) {
		pr_debug("Stopping Unplug Check Worker since USB is removed"
			"reg_loop = %d, fsm = %d ibat = %d\n",
			pm_chg_get_regulation_loop(chip),
			pm_chg_get_fsm_state(chip),
			get_prop_batt_current(chip)
			);
		return;
	}

	pm_chg_iusbmax_get(chip, &usb_ma);

#if !defined(CONFIG_MACH_MSM8960_L1m) && !defined (CONFIG_MACH_MSM8960_L1v)
	if (usb_ma == 500 && !usb_target_ma) {
		pr_debug("Stopping Unplug Check Worker since USB == 500mA\n");
		disable_input_voltage_regulation(chip);
		return;
	}
#endif

	if (usb_ma <= 100) {
		pr_debug(
			"Unenumerated yet or suspended usb_ma = %d skipping\n",
			usb_ma);
		goto check_again_later;
	}
	if (pm8921_chg_is_enabled(chip, CHG_GONE_IRQ))
		pr_debug("chg gone irq is enabled\n");

#if !defined (CONFIG_MACH_MSM8960_L1m) && !defined (CONFIG_MACH_MSM8960_L1v)
	reg_loop = pm_chg_get_regulation_loop(chip);
	pr_debug("reg_loop=0x%x usb_ma = %d\n", reg_loop, usb_ma);

	if ((reg_loop & VIN_ACTIVE_BIT) && (usb_ma > USB_WALL_THRESHOLD_MA)) {
		decrease_usb_ma_value(&usb_ma);
		usb_target_ma = usb_ma;
		/* end AICL here */
		__pm8921_charger_vbus_draw(usb_ma);
		printk("[dongju99.kim] usb_now=%d, usb_target = %d\n",
			usb_ma, usb_target_ma);
	}
#endif

	reg_loop = pm_chg_get_regulation_loop(chip);
	pr_debug("reg_loop=0x%x usb_ma = %d\n", reg_loop, usb_ma);

	if (reg_loop & VIN_ACTIVE_BIT) {
		ibat = get_prop_batt_current(chip);

		pr_debug("ibat = %d fsm = %d reg_loop = 0x%x\n",
				ibat, pm_chg_get_fsm_state(chip), reg_loop);
		if (ibat > 0) {
			int count = 0;

			while (count++ < param_vin_disable_counter
					&& usb_chg_plugged_in == 1) {
				attempt_reverse_boost_fix(chip, count, usb_ma);
				usb_chg_plugged_in
					= is_usb_chg_plugged_in(chip);
			}
		}
	}

	usb_chg_plugged_in = is_usb_chg_plugged_in(chip);
	chg_gone = pm_chg_get_rt_status(chip, CHG_GONE_IRQ);

	if (chg_gone == 1  && usb_chg_plugged_in == 1) {
		/* run the worker directly */
		pr_debug(" ver5 step: chg_gone=%d, usb_valid = %d\n",
						chg_gone, usb_chg_plugged_in);
#ifdef CONFIG_MACH_LGE_TEMP
		schedule_work(&chip->unplug_ovp_fet_open_work);
#else
		unplug_ovp_fet_open(chip);
#endif
	}

	if (!(reg_loop & VIN_ACTIVE_BIT)) {
		/* only increase iusb_max if vin loop not active */
		if (usb_ma < usb_target_ma) {
			increase_usb_ma_value(&usb_ma);
			__pm8921_charger_vbus_draw(usb_ma);
			pr_debug("usb_now=%d, usb_target = %d\n",
					usb_ma, usb_target_ma);
		} else {
			usb_target_ma = usb_ma;
		}
	}
check_again_later:
	/* schedule to check again later */
	schedule_delayed_work(&chip->unplug_check_work,
		      round_jiffies_relative(msecs_to_jiffies
				(UNPLUG_CHECK_WAIT_PERIOD_MS)));
}

static irqreturn_t loop_change_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	pr_debug("fsm_state=%d reg_loop=0x%x\n",
		pm_chg_get_fsm_state(data),
		pm_chg_get_regulation_loop(data));
	schedule_work(&chip->unplug_check_work.work);
	return IRQ_HANDLED;
}

static irqreturn_t fastchg_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;
	int high_transition;

	printk(KERN_INFO "[WIRELESS] FAST IRQ START\n");
	high_transition = pm_chg_get_rt_status(chip, FASTCHG_IRQ);
	pr_info("fastchg_irq : high_transition = %d", high_transition);
#ifdef CONFIG_LGE_WIRELESS_CHARGER
	if (!is_usb_chg_plugged_in(chip) && is_dc_chg_plugged_in(chip) && high_transition && !delayed_work_pending(&chip->eoc_work)) {
		wake_lock(&chip->eoc_wake_lock);
		schedule_delayed_work(&chip->eoc_work,round_jiffies_relative(msecs_to_jiffies(WIRELESS_EARLY_EOC_WORK_TIME)));
		printk(KERN_INFO "[WIRELESS] WLC fastchg_irq : eoc_work start ");
	}
#endif
	if (high_transition && !delayed_work_pending(&chip->eoc_work)) {
		wake_lock(&chip->eoc_wake_lock);
		pr_info("fastchg_irq : eoc_work start ");
		schedule_delayed_work(&chip->eoc_work,
				      round_jiffies_relative(msecs_to_jiffies
						     (EOC_CHECK_PERIOD_MS)));
	}
	power_supply_changed(&chip->batt_psy);
	bms_notify_check(chip);
	printk(KERN_INFO "[WIRELESS] FAST IRQ END\n");
	return IRQ_HANDLED;
}

static irqreturn_t trklchg_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	power_supply_changed(&chip->batt_psy);
	return IRQ_HANDLED;
}

static irqreturn_t batt_removed_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;
	int status;

#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
#ifdef CONFIG_LGE_WIRELESS_CHARGER
	if((is_usb_chg_plugged_in(chip) || wireless_is_plugged())
#else
	if(is_usb_chg_plugged_in(chip)
#endif
		&& !is_factory_cable() && !pseudo_batt_info.mode){
		pr_info("[PM] Battery removed but cable exists. Now reset as scenario !!\n");
		kernel_restart("battery");
	}
#endif

	status = pm_chg_get_rt_status(chip, BATT_REMOVED_IRQ);
	pr_debug("battery present=%d state=%d", !status,
					 pm_chg_get_fsm_state(data));
	handle_stop_ext_chg(chip);
	power_supply_changed(&chip->batt_psy);
	return IRQ_HANDLED;
}

static irqreturn_t batttemp_hot_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	handle_stop_ext_chg(chip);
	power_supply_changed(&chip->batt_psy);
	return IRQ_HANDLED;
}

static irqreturn_t chghot_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	pr_debug("Chg hot fsm_state=%d\n", pm_chg_get_fsm_state(data));
	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->usb_psy);
	handle_stop_ext_chg(chip);
	return IRQ_HANDLED;
}

static irqreturn_t batttemp_cold_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	pr_debug("Batt cold fsm_state=%d\n", pm_chg_get_fsm_state(data));
	handle_stop_ext_chg(chip);

	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->usb_psy);
	return IRQ_HANDLED;
}

static irqreturn_t chg_gone_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;
	int chg_gone, usb_chg_plugged_in;
#ifdef CONFIG_MACH_LGE_TEMP
	int rc;
	u8 reg;
#endif
	usb_chg_plugged_in = is_usb_chg_plugged_in(chip);
	chg_gone = pm_chg_get_rt_status(chip, CHG_GONE_IRQ);

	pr_debug("chg_gone=%d, usb_valid = %d\n", chg_gone, usb_chg_plugged_in);
	pr_debug("Chg gone fsm_state=%d\n", pm_chg_get_fsm_state(data));
#ifdef CONFIG_MACH_LGE_TEMP
       rc = pm8xxx_readb(chip->dev->parent, CHG_CNTRL_3, &reg);
       if (rc)
               pr_err("Failed to read CHG_CNTRL_3 rc=%d\n", rc);

       if (reg & CHG_USB_SUSPEND_BIT)
               return IRQ_HANDLED;
       schedule_work(&chip->unplug_ovp_fet_open_work);

#endif

	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->usb_psy);
	return IRQ_HANDLED;
}
/*
 *
 * bat_temp_ok_irq_handler - is edge triggered, hence it will
 * fire for two cases:
 *
 * If the interrupt line switches to high temperature is okay
 * and thus charging begins.
 * If bat_temp_ok is low this means the temperature is now
 * too hot or cold, so charging is stopped.
 *
 */
static irqreturn_t bat_temp_ok_irq_handler(int irq, void *data)
{
	int bat_temp_ok;
	struct pm8921_chg_chip *chip = data;

	bat_temp_ok = pm_chg_get_rt_status(chip, BAT_TEMP_OK_IRQ);

	pr_debug("batt_temp_ok = %d fsm_state%d\n",
			 bat_temp_ok, pm_chg_get_fsm_state(data));

	if (bat_temp_ok)
		handle_start_ext_chg(chip);
	else
		handle_stop_ext_chg(chip);

	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->usb_psy);
	bms_notify_check(chip);
	return IRQ_HANDLED;
}

static irqreturn_t coarse_det_low_irq_handler(int irq, void *data)
{
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t vdd_loop_irq_handler(int irq, void *data)
{
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t vreg_ov_irq_handler(int irq, void *data)
{
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t vbatdet_irq_handler(int irq, void *data)
{
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t batfet_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

#ifdef CONFIG_LGE_WIRELESS_CHARGER
	printk(KERN_INFO "[WIRELESS] BATFET RT=%d\n",pm_chg_get_rt_status(chip,BATFET_IRQ));
#else
	pr_debug("vreg ov\n");
	power_supply_changed(&chip->batt_psy);
#endif
	return IRQ_HANDLED;
}

#ifndef CONFIG_LGE_WIRELESS_CHARGER
static irqreturn_t dcin_valid_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;
	int dc_present;

	dc_present = pm_chg_get_rt_status(chip, DCIN_VALID_IRQ);
	if (chip->ext_psy)
		power_supply_set_online(chip->ext_psy, dc_present);
	chip->dc_present = dc_present;
	if (dc_present)
		handle_start_ext_chg(chip);
	else
		handle_stop_ext_chg(chip);
#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
	if (dc_present) {
		last_stop_charging = 0;
		pseudo_ui_charging = 0;
		/* 120406 mansu.lee@lge.com Implements Thermal Mitigation iUSB setting */
		pre_xo_mitigation  = 0;
		/* 120406 mansu.lee@lge.com*/
	}
#endif
	return IRQ_HANDLED;
}
#endif

static irqreturn_t dcin_ov_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	handle_stop_ext_chg(chip);
	return IRQ_HANDLED;
}

static irqreturn_t dcin_uv_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	handle_stop_ext_chg(chip);

	return IRQ_HANDLED;
}

#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
/* LGE_CHANGE
 * Change the charger_temp_scenatio
 * 2012-02-27, jooyeong.lee@lge.com
 */
static int chg_is_battery_too_hot_or_too_cold(void *data, int batt_temp, int batt_level)
{
	int batt_temp_level;
	int rtnValue = 0;
	struct pm8921_chg_chip *chip = data;
	pseudo_ui_charging = 0;



	if (batt_temp > chip->temp_level_1)
		batt_temp_level = CHG_BATT_TEMP_LEVEL_1;
	else if (batt_temp >= chip->temp_level_2)
		batt_temp_level = CHG_BATT_TEMP_LEVEL_2;
	else if (batt_temp >= chip->temp_level_3)
		batt_temp_level = CHG_BATT_TEMP_LEVEL_3;
	else if (batt_temp > chip->temp_level_4)
		batt_temp_level = CHG_BATT_TEMP_LEVEL_4;
	else if (batt_temp >= chip->temp_level_5)
		batt_temp_level = CHG_BATT_TEMP_LEVEL_5;
 	else
		batt_temp_level = CHG_BATT_TEMP_LEVEL_6;

#if defined(CONFIG_MACH_MSM8960_L_DCM) || defined(CONFIG_MACH_MSM8960_L1m)
	if((batt_temp_level == CHG_BATT_TEMP_LEVEL_1) || (batt_temp_level == CHG_BATT_TEMP_LEVEL_6))
#else
	if(batt_temp_level == CHG_BATT_TEMP_LEVEL_1)
#endif
		stop_charging_status = 1;
	else if((batt_temp_level == CHG_BATT_TEMP_LEVEL_4) || batt_temp_level == CHG_BATT_TEMP_LEVEL_5)
		stop_charging_status = 0;

	/* LGE_CHANGE
	 * add the xo_thermal mitigation way
	 * 2012-04-10, hiro.kwon@lge.com
	*/
	if((chip->thermal_mitigation_method == IUSB_USE_FOR_ISYSTEM_METHOD) &&
		(batt_temp_level != CHG_BATT_TEMP_LEVEL_1) &&
		(batt_temp_level != CHG_BATT_TEMP_LEVEL_6))
	{
		int ret;
		struct pm8xxx_adc_chan_result result;
		int temp;

		xo_mitigation_stop_chg = false;

		ret = pm8xxx_adc_read(CHANNEL_MUXOFF, &result);
		if (ret)
		{
			pr_err("error reading xo_therm adc channel = %d, ret = %d\n",
				CHANNEL_MUXOFF, ret);
			return rtnValue;
		}
		temp = result.physical;

		temp /= 1000;
		pr_info("[Thermal] xo_therm physical(%lld), temp(%d)", result.physical, temp );
		if ( temp >= 40 )
		{
        		rtnValue = 1;
			pseudo_ui_charging = 1;
			chg_batt_temp_state = CHG_BATT_STOP_CHARGING_STATE;
			xo_mitigation_stop_chg = true;
			pr_info("[Thermal]xo_therm 40 over! stop charging!!");
			return rtnValue;
		}

	}
	/* 2012-04-10, hiro.kwon@lge.com */

    printk(KERN_INFO "[PM] hot or cold :state=%d, level=%d , TEMP=%d , VOLT=%d\n",chg_batt_temp_state, batt_temp_level, batt_temp, batt_level);

	switch(chg_batt_temp_state)
	{
	case CHG_BATT_NORMAL_STATE:
		if (batt_temp_level == CHG_BATT_TEMP_LEVEL_1 || batt_temp_level == CHG_BATT_TEMP_LEVEL_6 ||
			(batt_temp_level == CHG_BATT_TEMP_LEVEL_2 && batt_level > 4000)) {

			chg_batt_temp_state = CHG_BATT_STOP_CHARGING_STATE;
			/*
			 * kiwone.seo@lge.com 2011-0609
			 * for show charging ani.although charging is stopped : charging scenario
			 */
			if (batt_temp_level == CHG_BATT_TEMP_LEVEL_2 && batt_level > 4000) {
				/* we must show charging image although charging is stopped. */
				pseudo_ui_charging = 1;
			}

/* BEGIN: pk.jeon@lge.com 2012-04-23
 * For Use Battery Health in case BTM Disable in Domestic Operator and DCM (D1LU, D1LSK, D1LKT, L_DCM) */
#if defined(CONFIG_MACH_MSM8960_D1LU) || defined(CONFIG_MACH_MSM8960_D1LSK) || defined(CONFIG_MACH_MSM8960_D1LKT) || defined(CONFIG_MACH_MSM8960_L_DCM) || defined(CONFIG_MACH_MSM8960_L1sk) || defined(CONFIG_MACH_MSM8960_L1kt) || defined(CONFIG_MACH_MSM8960_L1u)
			if(batt_temp_level == CHG_BATT_TEMP_LEVEL_1){
				pr_debug("Over Temperature 55 batt_health -> POWER_SUPPLY_HEALTH_OVERHEAT\n");
				batt_health=POWER_SUPPLY_HEALTH_OVERHEAT;
			}
			else if(batt_temp_level == CHG_BATT_TEMP_LEVEL_6){
				pr_debug("Under Temperature -10 batt_health -> POWER_SUPPLY_HEALTH_COLD\n");
				batt_health=POWER_SUPPLY_HEALTH_COLD;
			}
#endif
/* END : pk.jeon@lge.com 2012-04-23 */
			printk(KERN_INFO "[PM] BATT TEMP OUT OF SPEC : NORMAL ---> STOP , TEMP=%d , VOLT=%d\n",
				batt_temp, batt_level);
			rtnValue = 1;
		}
		else if (batt_temp_level == CHG_BATT_TEMP_LEVEL_2 && batt_level <= 4000) {
			chg_batt_temp_state = CHG_BATT_DC_CURRENT_STATE;

			printk(KERN_INFO "[PM] BATT TEMP 45 ~ 55 : NORMAL ---> DECREASE , TEMP=%d , VOLT=%d , reduce IBAT=%d\n",
			batt_temp, batt_level, PM8921_CHG_IBATMAX_MIN);
	
			rtnValue = 0;

#if defined(CONFIG_MACH_MSM8960_L1m)
			pm8921_charger_vbus_draw_internal(USB_WALL_THRESHOLD_MA);
			usb_target_ma = USB_WALL_THRESHOLD_MA; 
#else
			if (the_chip)
				pm_chg_ibatmax_set(chip, PM8921_CHG_IBATMAX_MIN);
#endif
			#ifdef CONFIG_LGE_WIRELESS_CHARGER
			if(is_dc_chg_plugged_in(chip)) {
				wireless_current_set(chip);
			}
			#endif
		}
#ifdef CONFIG_MACH_MSM8960_L_DCM
		else if (batt_temp_level == CHG_BATT_TEMP_LEVEL_5) {
			chg_batt_temp_state = CHG_BATT_DC_CURRENT_STATE;

			printk(KERN_INFO "[PM] BATT TEMP -10 ~ 0 : NORMAL ---> DECREASE , TEMP=%d , VOLT=%d , reduce IBAT=%d\n",
				batt_temp, batt_level, PM8921_CHG_IBATMAX_MIN);

			rtnValue = 0;
			if (the_chip)
				pm_chg_ibatmax_set(chip, PM8921_CHG_IBATMAX_MIN);

		}
#endif
		else {
			chg_batt_temp_state = CHG_BATT_NORMAL_STATE;
			printk(KERN_INFO "[PM] BATT TEMP NORMAL : NORMAL ---> NORMAL , TEMP=%d , VOLT=%d\n",
				batt_temp, batt_level);
			rtnValue = 0;
		}
		break;

	case CHG_BATT_DC_CURRENT_STATE:
		if (batt_temp_level == CHG_BATT_TEMP_LEVEL_1 || batt_temp_level == CHG_BATT_TEMP_LEVEL_6 ||
			(batt_temp_level == CHG_BATT_TEMP_LEVEL_2 && batt_level > 4000)) {

			chg_batt_temp_state = CHG_BATT_STOP_CHARGING_STATE;
			if (batt_temp_level == CHG_BATT_TEMP_LEVEL_2 && batt_level > 4000) {
				/* we must show charging image although charging is stopped. */
				pseudo_ui_charging = 1;
			}

/* BEGIN: pk.jeon@lge.com 2012-04-23
 * For Use Battery Health in case BTM Disable in Domestic Operator and DCM (D1LU, D1LSK, D1LKT, L_DCM) */
#if defined(CONFIG_MACH_MSM8960_D1LU) || defined(CONFIG_MACH_MSM8960_D1LSK) || defined(CONFIG_MACH_MSM8960_D1LKT) || defined(CONFIG_MACH_MSM8960_L_DCM) || defined(CONFIG_MACH_MSM8960_L1sk) || defined(CONFIG_MACH_MSM8960_L1kt) || defined(CONFIG_MACH_MSM8960_L1u)
			if(batt_temp_level == CHG_BATT_TEMP_LEVEL_1){
				pr_debug("Over Temperature 55 batt_health -> POWER_SUPPLY_HEALTH_OVERHEAT\n");
				batt_health=POWER_SUPPLY_HEALTH_OVERHEAT;
			}
			else if(batt_temp_level == CHG_BATT_TEMP_LEVEL_6) {
				pr_debug("Under Temperature -10 batt_health -> POWER_SUPPLY_HEALTH_COLD\n");
				batt_health=POWER_SUPPLY_HEALTH_COLD;
			}
#endif
/* END : pk.jeon@lge.com 2012-04-23 */
			printk(KERN_INFO "[PM] BATT TEMP OUT OF SPEC : DECREASE ---> STOP , TEMP=%d , VOLT=%d\n",
				batt_temp, batt_level);
			rtnValue = 1;
		}
		else if (batt_temp_level == CHG_BATT_TEMP_LEVEL_2 && batt_level <= 4000) {
			chg_batt_temp_state = CHG_BATT_DC_CURRENT_STATE;

			printk(KERN_INFO "[PM] BATT TEMP 45 ~ 55 : DECREASE ---> DECREASE , TEMP=%d , VOLT=%d\n",
				batt_temp, batt_level);

			rtnValue = 0;
#if defined(CONFIG_MACH_MSM8960_L1m)
			pm8921_charger_vbus_draw_internal(USB_WALL_THRESHOLD_MA);
			usb_target_ma = USB_WALL_THRESHOLD_MA; 
#else
			pm_chg_ibatmax_set(chip, PM8921_CHG_IBATMAX_MIN);
#endif

		}
#ifdef CONFIG_MACH_MSM8960_L_DCM
		else if (batt_temp_level == CHG_BATT_TEMP_LEVEL_5) {
			chg_batt_temp_state = CHG_BATT_DC_CURRENT_STATE;

			printk(KERN_INFO "[PM] BATT TEMP -10 ~ 0 : DECREASE ---> DECREASE , TEMP=%d , VOLT=%d\n",
				batt_temp, batt_level);

			rtnValue = 0;
		}
		else if (batt_temp_level == CHG_BATT_TEMP_LEVEL_4 || batt_temp_level == CHG_BATT_TEMP_LEVEL_3) {
			chg_batt_temp_state = CHG_BATT_NORMAL_STATE;
			printk(KERN_INFO "[PM] BATT TEMP NORMAL : DECREASE ---> NORMAL , TEMP=%d , VOLT=%d , Restore IBAT=%d\n",
				batt_temp, batt_level, chip->max_bat_chg_current);
			rtnValue = 0;
			if (the_chip)
				pm_chg_ibatmax_set(chip, chip->max_bat_chg_current);
		}
#else
		else if (batt_temp_level == CHG_BATT_TEMP_LEVEL_4 || batt_temp_level == CHG_BATT_TEMP_LEVEL_5 ||
				batt_temp_level == CHG_BATT_TEMP_LEVEL_3) {
			chg_batt_temp_state = CHG_BATT_NORMAL_STATE;
			printk(KERN_INFO "[PM] BATT TEMP NORMAL : DECREASE ---> NORMAL , TEMP=%d , VOLT=%d , Restore IBAT=%d\n",
				batt_temp, batt_level, chip->max_bat_chg_current);
			rtnValue = 0;
			if (the_chip)
			{
				pm_chg_ibatmax_set(chip, chip->max_bat_chg_current);
				usb_target_ma = C_NONE_TA_MA; 
			}
			#ifdef CONFIG_LGE_WIRELESS_CHARGER
			if(is_dc_chg_plugged_in(chip)) {
				wireless_current_set(chip);
			}
			#endif
		}
#endif
		else {
			chg_batt_temp_state = CHG_BATT_DC_CURRENT_STATE;
			printk(KERN_INFO "[PM] BATT TEMP UNREAL : DECREASE ---> DECREASE , TEMP=%d , VOLT=%d\n",
				batt_temp, batt_level);
			rtnValue = 0;
#if defined(CONFIG_MACH_MSM8960_L1m)
			pm8921_charger_vbus_draw_internal(USB_WALL_THRESHOLD_MA);
			usb_target_ma = USB_WALL_THRESHOLD_MA; 
#endif
		}
		break;

	case CHG_BATT_STOP_CHARGING_STATE:
#if defined(CONFIG_MACH_MSM8960_L_DCM)
			if(batt_temp_level == CHG_BATT_TEMP_LEVEL_1){
				pr_debug("Over Temperature 55 batt_health -> POWER_SUPPLY_HEALTH_OVERHEAT\n");
				batt_health=POWER_SUPPLY_HEALTH_OVERHEAT;
			}
			else if(batt_temp_level == CHG_BATT_TEMP_LEVEL_6){
				pr_debug("Under Temperature -10 batt_health -> POWER_SUPPLY_HEALTH_COLD\n");
				batt_health=POWER_SUPPLY_HEALTH_COLD;
			}
#endif

		if (batt_temp_level == CHG_BATT_TEMP_LEVEL_4) {
			chg_batt_temp_state = CHG_BATT_NORMAL_STATE;
			printk(KERN_INFO "[PM] BATT TEMP NORMAL : STOP ---> NORMAL , TEMP=%d , VOLT=%d , Restore IBAT=%d\n",
				batt_temp,batt_level, chip->max_bat_chg_current);
			rtnValue = 0;
			if (the_chip)
				pm_chg_ibatmax_set(chip, chip->max_bat_chg_current);

			#ifdef CONFIG_LGE_WIRELESS_CHARGER
			if(is_dc_chg_plugged_in(chip)) {
				wireless_current_set(chip);
			}
			#endif
/* BEGIN: pk.jeon@lge.com 2012-04-23
 * For Use Battery Health in case BTM Disable in Domestic Operator and DCM (D1LU, D1LSK, D1LKT, L_DCM) */
#if defined(CONFIG_MACH_MSM8960_D1LU) || defined(CONFIG_MACH_MSM8960_D1LSK) || defined(CONFIG_MACH_MSM8960_D1LKT) || defined(CONFIG_MACH_MSM8960_L_DCM) || defined(CONFIG_MACH_MSM8960_L1sk) || defined(CONFIG_MACH_MSM8960_L1kt) || defined(CONFIG_MACH_MSM8960_L1u)
				pr_debug("Good Temperature -4~41 batt_health -> POWER_SUPPLY_HEALTH_GOOD\n");
				batt_health=POWER_SUPPLY_HEALTH_GOOD;
#endif
/* END : pk.jeon@lge.com 2012-04-23 */

		}
		/*
		 * kiwone.seo@lge.com 2011-05-12, charging scenario. do not anything.
		 */

#if defined(CONFIG_MACH_MSM8960_L1m) // added in L1m
		else if ((batt_temp_level == CHG_BATT_TEMP_LEVEL_2 && batt_level <= 4000) ||
					(batt_temp_level == CHG_BATT_TEMP_LEVEL_3)) {
			chg_batt_temp_state = CHG_BATT_DC_CURRENT_STATE;
			printk(KERN_INFO "[PM] BATT TEMP 45 ~ 55 : STOP ---> DECREASE , TEMP=%d , VOLT=%d , reduce IBAT=%d\n",
				batt_temp, batt_level, PM8921_CHG_IBATMAX_MIN);

			rtnValue = 0;

			pm8921_charger_vbus_draw_internal(USB_WALL_THRESHOLD_MA);
			usb_target_ma = USB_WALL_THRESHOLD_MA; 
		}
#endif

#ifdef CONFIG_LGE_WIRELESS_CHARGER
		else if(batt_temp_level == CHG_BATT_TEMP_LEVEL_3 && is_dc_chg_plugged_in(chip)) {
			chg_batt_temp_state = CHG_BATT_DC_CURRENT_STATE;
			printk(KERN_INFO "[PM][WIRELESS] BATT TEMP NORMAL : STOP ---> DECREASE , TEMP=%d , VOLT=%d\n",batt_temp,batt_level);
			rtnValue = 0;
			wireless_current_set(chip);
			#if defined(CONFIG_MACH_MSM8960_D1LU) || defined(CONFIG_MACH_MSM8960_D1LSK) || defined(CONFIG_MACH_MSM8960_D1LKT) || defined(CONFIG_MACH_MSM8960_L_DCM) || defined(CONFIG_MACH_MSM8960_L1sk) || defined(CONFIG_MACH_MSM8960_L1kt) || defined(CONFIG_MACH_MSM8960_L1u)
			printk(KERN_INFO "[WIRELESS] batt_health -> POWER_SUPPLY_HEALTH_GOOD\n");
			batt_health=POWER_SUPPLY_HEALTH_GOOD;
			#endif
		}
#endif
		else {
			chg_batt_temp_state = CHG_BATT_STOP_CHARGING_STATE;
			/*
			 * kiwone.seo@lge.com 2011-0621
			 * 1. stop charging
			 * 2. remove charger
			 * 3 insert charger while stop charging
			 */
			if (batt_temp_level == CHG_BATT_TEMP_LEVEL_2 && batt_level > 4000) {
				 /* we must show charging image although charging is stopped. */
				pseudo_ui_charging = 1;
			}
			printk(KERN_INFO "[PM] BATT TEMP OUT OF SPEC : STOP ---> STOP , TEMP=%d , VOLT=%d\n",batt_temp,batt_level);
			rtnValue = 1;
		}
		break;
	}

	return rtnValue;
}

#endif

#ifndef CONFIG_LGE_WIRELESS_CHARGER
static int __pm_batt_external_power_changed_work(struct device *dev, void *data)
{
	struct power_supply *psy = &the_chip->batt_psy;
	struct power_supply *epsy = dev_get_drvdata(dev);
	int i, dcin_irq;

	/* Only search for external supply if none is registered */
	if (!the_chip->ext_psy) {
		dcin_irq = the_chip->pmic_chg_irq[DCIN_VALID_IRQ];
		for (i = 0; i < epsy->num_supplicants; i++) {
			if (!strncmp(epsy->supplied_to[i], psy->name, 7)) {
				if (!strncmp(epsy->name, "dc", 2)) {
					the_chip->ext_psy = epsy;
					dcin_valid_irq_handler(dcin_irq,
							the_chip);
				}
			}
		}
	}
	return 0;
}

static void pm_batt_external_power_changed(struct power_supply *psy)
{
	/* Only look for an external supply if it hasn't been registered */
	if (!the_chip->ext_psy)
		class_for_each_device(power_supply_class, NULL, psy,
					 __pm_batt_external_power_changed_work);
}
#endif

/**
 * update_heartbeat - internal function to update userspace
 *		per update_time minutes
 *
 */
#define LOW_SOC_HEARTBEAT_MS	20000
static void update_heartbeat(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct pm8921_chg_chip *chip = container_of(dwork,
				struct pm8921_chg_chip, update_heartbeat_work);

	pm_chg_failed_clear(chip, 1);
	power_supply_changed(&chip->batt_psy);
	if (chip->recent_reported_soc <= 20)
		schedule_delayed_work(&chip->update_heartbeat_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (LOW_SOC_HEARTBEAT_MS)));
	else
		schedule_delayed_work(&chip->update_heartbeat_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (chip->update_time)));
}
#define VDD_LOOP_ACTIVE_BIT	BIT(3)
#define VDD_MAX_INCREASE_MV	400
static int vdd_max_increase_mv = VDD_MAX_INCREASE_MV;
module_param(vdd_max_increase_mv, int, 0644);

static int ichg_threshold_ua = -400000;
module_param(ichg_threshold_ua, int, 0644);

#define PM8921_CHG_VDDMAX_RES_MV	10
static void adjust_vdd_max_for_fastchg(struct pm8921_chg_chip *chip)
{
	int ichg_meas_ua, vbat_uv;
	int ichg_meas_ma;
	int adj_vdd_max_mv, programmed_vdd_max;
	int vbat_batt_terminal_uv;
	int vbat_batt_terminal_mv;
	int reg_loop;
	int delta_mv = 0;

	if (chip->rconn_mohm == 0) {
#ifdef CONFIG_LGE_PM
		printk(KERN_INFO "Exiting as rconn_mohm is 0\n");
#else
		pr_debug("Exiting as rconn_mohm is 0\n");
#endif
		return;
	}
	/* adjust vdd_max only in normal temperature zone */
	if (chip->is_bat_cool || chip->is_bat_warm) {
		pr_debug("Exiting is_bat_cool = %d is_batt_warm = %d\n",
				chip->is_bat_cool, chip->is_bat_warm);
		return;
	}

	reg_loop = pm_chg_get_regulation_loop(chip);
	if (!(reg_loop & VDD_LOOP_ACTIVE_BIT)) {
		pr_debug("Exiting Vdd loop is not active reg loop=0x%x\n",
			reg_loop);
		return;
	}

	pm8921_bms_get_simultaneous_battery_voltage_and_current(&ichg_meas_ua,
								&vbat_uv);
	if (ichg_meas_ua >= 0) {
		pr_debug("Exiting ichg_meas_ua = %d > 0\n", ichg_meas_ua);
		return;
	}
	if (ichg_meas_ua <= ichg_threshold_ua) {
		pr_debug("Exiting ichg_meas_ua = %d < ichg_threshold_ua = %d\n",
					ichg_meas_ua, ichg_threshold_ua);
		return;
	}
	ichg_meas_ma = ichg_meas_ua / 1000;

	/* rconn_mohm is in milliOhms */
	vbat_batt_terminal_uv = vbat_uv + ichg_meas_ma * the_chip->rconn_mohm;
	vbat_batt_terminal_mv = vbat_batt_terminal_uv/1000;
	pm_chg_vddmax_get(the_chip, &programmed_vdd_max);

	delta_mv =  chip->max_voltage_mv - vbat_batt_terminal_mv;

	adj_vdd_max_mv = programmed_vdd_max + delta_mv;
	pr_debug("vdd_max needs to be changed by %d mv from %d to %d\n",
			delta_mv,
			programmed_vdd_max,
			adj_vdd_max_mv);

	if (adj_vdd_max_mv < chip->max_voltage_mv) {
		pr_debug("adj vdd_max lower than default max voltage\n");
		return;
	}

	adj_vdd_max_mv = DIV_ROUND_UP(adj_vdd_max_mv, PM8921_CHG_VDDMAX_RES_MV)
					* PM8921_CHG_VDDMAX_RES_MV;

	if (adj_vdd_max_mv > (chip->max_voltage_mv + vdd_max_increase_mv))
		adj_vdd_max_mv = chip->max_voltage_mv + vdd_max_increase_mv;
	pr_debug("adjusting vdd_max_mv to %d to make "
		"vbat_batt_termial_uv = %d to %d\n",
		adj_vdd_max_mv, vbat_batt_terminal_uv, chip->max_voltage_mv);
	pm_chg_vddmax_set(chip, adj_vdd_max_mv);
}

enum {
	CHG_IN_PROGRESS,
	CHG_NOT_IN_PROGRESS,
	CHG_FINISHED,
};

#if defined(CONFIG_MACH_MSM8960_D1LU) || defined(CONFIG_MACH_MSM8960_D1LSK) || defined(CONFIG_MACH_MSM8960_D1LKT) || defined(CONFIG_MACH_MSM8960_L1sk) || defined(CONFIG_MACH_MSM8960_L1kt) || defined(CONFIG_MACH_MSM8960_L1u)
#define VBAT_TOLERANCE_MV	160
#elif defined(CONFIG_MACH_MSM8960_D1LV)
#define VBAT_TOLERANCE_MV       60
#elif defined(CONFIG_MACH_MSM8960_L0)
#define VBAT_TOLERANCE_MV	80			/* 4.28V = 4.36V(vddmax) - 0.08V(tolerance) */
#else
#define VBAT_TOLERANCE_MV	70
#endif
#define CHG_DISABLE_MSLEEP	100
static int is_charging_finished(struct pm8921_chg_chip *chip)
{
	int vbat_meas_uv, vbat_meas_mv, vbat_programmed, vbatdet_low;
	int ichg_meas_ma, iterm_programmed;
	int regulation_loop, fast_chg, vcp;
	int rc;
	static int last_vbat_programmed = -EINVAL;

#if defined(CONFIG_LGE_PM) && !defined(CONFIG_BATTERY_MAX17043)
	int soc;
#endif

	if (!is_ext_charging(chip)) {
		/* return if the battery is not being fastcharged */
		fast_chg = pm_chg_get_rt_status(chip, FASTCHG_IRQ);
		pr_debug("fast_chg = %d\n", fast_chg);
		if (fast_chg == 0)
			return CHG_NOT_IN_PROGRESS;

		vcp = pm_chg_get_rt_status(chip, VCP_IRQ);
		pr_debug("vcp = %d\n", vcp);
		if (vcp == 1)
			return CHG_IN_PROGRESS;

		vbatdet_low = pm_chg_get_rt_status(chip, VBATDET_LOW_IRQ);
		pr_debug("vbatdet_low = %d\n", vbatdet_low);
		if (vbatdet_low == 1)
			return CHG_IN_PROGRESS;

		/* reset count if battery is hot/cold */
		rc = pm_chg_get_rt_status(chip, BAT_TEMP_OK_IRQ);
		pr_debug("batt_temp_ok = %d\n", rc);
		if (rc == 0)
			return CHG_IN_PROGRESS;

		/* reset count if battery voltage is less than vddmax */
		vbat_meas_uv = get_prop_battery_uvolts(chip);
		if (vbat_meas_uv < 0)
			return CHG_IN_PROGRESS;
		vbat_meas_mv = vbat_meas_uv / 1000;

		rc = pm_chg_vddmax_get(chip, &vbat_programmed);
		if (rc) {
			pr_err("couldnt read vddmax rc = %d\n", rc);
			return CHG_IN_PROGRESS;
		}
		pr_debug("vddmax = %d vbat_meas_mv=%d\n",
			 vbat_programmed, vbat_meas_mv);
		if (vbat_meas_mv < vbat_programmed - VBAT_TOLERANCE_MV)
			return CHG_IN_PROGRESS;

		if (last_vbat_programmed == -EINVAL)
			last_vbat_programmed = vbat_programmed;
		if (last_vbat_programmed !=  vbat_programmed) {
			/* vddmax changed, reset and check again */
			pr_debug("vddmax = %d last_vdd_max=%d\n",
				 vbat_programmed, last_vbat_programmed);
			last_vbat_programmed = vbat_programmed;
			return CHG_IN_PROGRESS;
		}

		regulation_loop = pm_chg_get_regulation_loop(chip);
		if (regulation_loop < 0) {
			pr_err("couldnt read the regulation loop err=%d\n",
				regulation_loop);
			return CHG_IN_PROGRESS;
		}
		pr_debug("regulation_loop=%d\n", regulation_loop);

		if (regulation_loop != 0 && regulation_loop != VDD_LOOP)
			return CHG_IN_PROGRESS;
	} /* !is_ext_charging */

	/* reset count if battery chg current is more than iterm */
	rc = pm_chg_iterm_get(chip, &iterm_programmed);
	if (rc) {
		pr_err("couldnt read iterm rc = %d\n", rc);
		return CHG_IN_PROGRESS;
	}

	ichg_meas_ma = (get_prop_batt_current(chip)) / 1000;
	pr_debug("iterm_programmed = %d ichg_meas_ma=%d\n",
				iterm_programmed, ichg_meas_ma);
	/*
	 * ichg_meas_ma < 0 means battery is drawing current
	 * ichg_meas_ma > 0 means battery is providing current
	 */
	if (ichg_meas_ma > 0)
		return CHG_IN_PROGRESS;

	if (ichg_meas_ma * -1 > iterm_programmed)
		return CHG_IN_PROGRESS;

/* BMS, Add SOC condition to check charging complete.
 * In low temperature, as the battery internal resistance increase rapidly, charging is completed early than room temperature.
 * To guarantee minimum 96% charging in all temperature range, add soc condition.
 * 2012-06-09, junsin.park@lge.com
 */
#if defined(CONFIG_LGE_PM) && !defined(CONFIG_BATTERY_MAX17043)
		soc = get_prop_batt_capacity(chip);
		if (soc < 96)
			return CHG_IN_PROGRESS;
#endif

	return CHG_FINISHED;
}

/**
 * eoc_worker - internal function to check if battery EOC
 *		has happened
 *
 * If all conditions favouring, if the charge current is
 * less than the term current for three consecutive times
 * an EOC has happened.
 * The wakelock is released if there is no need to reshedule
 * - this happens when the battery is removed or EOC has
 * happened
 */
#define CONSECUTIVE_COUNT	3
static void eoc_worker(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct pm8921_chg_chip *chip = container_of(dwork,
				struct pm8921_chg_chip, eoc_work);
	static int count;
	int end;

	pm_chg_failed_clear(chip, 1);
#ifdef CONFIG_LGE_WIRELESS_CHARGER
	if (is_usb_chg_plugged_in(chip)) {
		printk(KERN_INFO "eoc_worker\n");
		end = is_charging_finished(chip);
	}
	else if(is_dc_chg_plugged_in(chip)) {
		printk(KERN_INFO "[WIRELESS] eoc_worker : WLC ====== \n");
		end = wireless_is_charging_finished(chip);
	}
	else {
		printk(KERN_INFO "eoc_worker : No Charger\n");
		end = CHG_NOT_IN_PROGRESS;
	}
#else
	end = is_charging_finished(chip);
#endif

	if (end == CHG_NOT_IN_PROGRESS) {
		count = 0;
		wake_unlock(&chip->eoc_wake_lock);
		return;
	}

	if (end == CHG_FINISHED) {
		count++;
	} else {
		count = 0;
	}

	if (count == CONSECUTIVE_COUNT) {
		count = 0;
		pr_info("End of Charging\n");

#ifdef CONFIG_LGE_WIRELESS_CHARGER
		wireless_chip_control_worker(&chip->wireless_chip_control_work.work);
		wireless_charge_done = true;
#endif
		pm_chg_auto_enable(chip, 0);

		if (is_ext_charging(chip))
			chip->ext_charge_done = true;

		if (chip->is_bat_warm || chip->is_bat_cool)
			chip->bms_notify.is_battery_full = 0;
		else
			chip->bms_notify.is_battery_full = 1;
		/* declare end of charging by invoking chgdone interrupt */
		chgdone_irq_handler(chip->pmic_chg_irq[CHGDONE_IRQ], chip);
		wake_unlock(&chip->eoc_wake_lock);
	} else {
		adjust_vdd_max_for_fastchg(chip);
		pr_debug("EOC count = %d\n", count);
		schedule_delayed_work(&chip->eoc_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (EOC_CHECK_PERIOD_MS)));
	}
}

static void btm_configure_work(struct work_struct *work)
{
	int rc;

	rc = pm8xxx_adc_btm_configure(&btm_config);
	if (rc)
		pr_err("failed to configure btm rc=%d", rc);
}

DECLARE_WORK(btm_config_work, btm_configure_work);

static void set_appropriate_battery_current(struct pm8921_chg_chip *chip)
{
	unsigned int chg_current = chip->max_bat_chg_current;

	if (chip->is_bat_cool)
		chg_current = min(chg_current, chip->cool_bat_chg_current);

	if (chip->is_bat_warm)
		chg_current = min(chg_current, chip->warm_bat_chg_current);

	if (thermal_mitigation != 0 && chip->thermal_mitigation)
		chg_current = min(chg_current,
				chip->thermal_mitigation[thermal_mitigation]);

	pm_chg_ibatmax_set(the_chip, chg_current);
}

#define TEMP_HYSTERISIS_DEGC 2
static void battery_cool(bool enter)
{
	pr_debug("enter = %d\n", enter);
	if (enter == the_chip->is_bat_cool)
		return;
	the_chip->is_bat_cool = enter;
	if (enter) {
		btm_config.low_thr_temp =
			the_chip->cool_temp_dc + TEMP_HYSTERISIS_DEGC;
		set_appropriate_battery_current(the_chip);
		pm_chg_vddmax_set(the_chip, the_chip->cool_bat_voltage);
		pm_chg_vbatdet_set(the_chip,
			the_chip->cool_bat_voltage
			- the_chip->resume_voltage_delta);
	} else {
		btm_config.low_thr_temp = the_chip->cool_temp_dc;
		set_appropriate_battery_current(the_chip);
		pm_chg_vddmax_set(the_chip, the_chip->max_voltage_mv);
		pm_chg_vbatdet_set(the_chip,
			the_chip->max_voltage_mv
			- the_chip->resume_voltage_delta);
	}
	schedule_work(&btm_config_work);
}

static void battery_warm(bool enter)
{
	pr_debug("enter = %d\n", enter);
	if (enter == the_chip->is_bat_warm)
		return;
	the_chip->is_bat_warm = enter;
	if (enter) {
		btm_config.high_thr_temp =
			the_chip->warm_temp_dc - TEMP_HYSTERISIS_DEGC;
		set_appropriate_battery_current(the_chip);
		pm_chg_vddmax_set(the_chip, the_chip->warm_bat_voltage);
		pm_chg_vbatdet_set(the_chip,
			the_chip->warm_bat_voltage
			- the_chip->resume_voltage_delta);
	} else {
		btm_config.high_thr_temp = the_chip->warm_temp_dc;
		set_appropriate_battery_current(the_chip);
		pm_chg_vddmax_set(the_chip, the_chip->max_voltage_mv);
		pm_chg_vbatdet_set(the_chip,
			the_chip->max_voltage_mv
			- the_chip->resume_voltage_delta);
	}
	schedule_work(&btm_config_work);
}

static int configure_btm(struct pm8921_chg_chip *chip)
{
	int rc;

	if (chip->warm_temp_dc != INT_MIN)
		btm_config.btm_warm_fn = battery_warm;
	else
		btm_config.btm_warm_fn = NULL;

	if (chip->cool_temp_dc != INT_MIN)
		btm_config.btm_cool_fn = battery_cool;
	else
		btm_config.btm_cool_fn = NULL;

	btm_config.low_thr_temp = chip->cool_temp_dc;
	btm_config.high_thr_temp = chip->warm_temp_dc;
	btm_config.interval = chip->temp_check_period;
	rc = pm8xxx_adc_btm_configure(&btm_config);
	if (rc)
		pr_err("failed to configure btm rc = %d\n", rc);
	rc = pm8xxx_adc_btm_start();
	if (rc)
		pr_err("failed to start btm rc = %d\n", rc);

	return rc;
}

/**
 * set_disable_status_param -
 *
 * Internal function to disable battery charging and also disable drawing
 * any current from the source. The device is forced to run on a battery
 * after this.
 */
static int set_disable_status_param(const char *val, struct kernel_param *kp)
{
	int ret;
	struct pm8921_chg_chip *chip = the_chip;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}
	pr_info("factory set disable param to %d\n", charging_disabled);
	if (chip) {
		pm_chg_auto_enable(chip, !charging_disabled);
		pm_chg_charge_dis(chip, charging_disabled);
#if defined (CONFIG_MACH_MSM8960_VU2SK) || defined (CONFIG_MACH_MSM8960_VU2U) || defined (CONFIG_MACH_MSM8960_VU2KT) || defined (CONFIG_MACH_MSM8960_L1m)
        printk("%s%d\n", __func__, charging_disabled);
#endif
	}
	return 0;
}
module_param_call(disabled, set_disable_status_param, param_get_uint,
					&charging_disabled, 0644);

static int rconn_mohm;
static int set_rconn_mohm(const char *val, struct kernel_param *kp)
{
	int ret;
	struct pm8921_chg_chip *chip = the_chip;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}
	if (chip)
		chip->rconn_mohm = rconn_mohm;
	return 0;
}
module_param_call(rconn_mohm, set_rconn_mohm, param_get_uint,
					&rconn_mohm, 0644);
/**
 * set_thermal_mitigation_level -
 *
 * Internal function to control battery charging current to reduce
 * temperature
 */
static int set_therm_mitigation_level(const char *val, struct kernel_param *kp)
{
	int ret;
	struct pm8921_chg_chip *chip = the_chip;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}

	if (!chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (!chip->thermal_mitigation) {
		pr_err("no thermal mitigation\n");
		return -EINVAL;
	}
	/* 120406 mansu.lee@lge.com Implements Thermal Mitigation iUSB setting */
#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
	/*
	* mansu.lee@lge.com
	* do nothing, we just wanna get "thermal_mitigation" values.
	* we will use this value to set charging current setting.
	* and it should not operate origin code.
	*/
	pr_debug("[Thermal] LGE current mitigation reflected\n");
#else
	if (thermal_mitigation < 0
		|| thermal_mitigation >= chip->thermal_levels) {
		pr_err("out of bound level selected\n");
		return -EINVAL;
	}

	set_appropriate_battery_current(chip);
#endif /* QCT ORIGIN*/
	/* 120406 mansu.lee@lge.com */
	return ret;
}
module_param_call(thermal_mitigation, set_therm_mitigation_level,
					param_get_uint,
					&thermal_mitigation, 0644);

static int set_usb_max_current(const char *val, struct kernel_param *kp)
{
	int ret, mA;
	struct pm8921_chg_chip *chip = the_chip;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}
	if (chip) {
		pr_warn("setting current max to %d\n", usb_max_current);
		pm_chg_iusbmax_get(chip, &mA);
		if (mA > usb_max_current)
			pm8921_charger_vbus_draw(usb_max_current);
		return 0;
	}
	return -EINVAL;
}
module_param_call(usb_max_current, set_usb_max_current,
	param_get_uint, &usb_max_current, 0644);

static void free_irqs(struct pm8921_chg_chip *chip)
{
	int i;

	for (i = 0; i < PM_CHG_MAX_INTS; i++)
		if (chip->pmic_chg_irq[i]) {
			free_irq(chip->pmic_chg_irq[i], chip);
			chip->pmic_chg_irq[i] = 0;
		}
}

/* determines the initial present states */
static void __devinit determine_initial_state(struct pm8921_chg_chip *chip)
{
	unsigned long flags;
	int fsm_state;

#ifndef CONFIG_LGE_WIRELESS_CHARGER
	chip->dc_present = !!is_dc_chg_plugged_in(chip);
#endif
	chip->usb_present = !!is_usb_chg_plugged_in(chip);

	notify_usb_of_the_plugin_event(chip->usb_present);
	if (chip->usb_present) {
		schedule_delayed_work(&chip->unplug_check_work,
			round_jiffies_relative(msecs_to_jiffies
				(UNPLUG_CHECK_WAIT_PERIOD_MS)));
		pm8921_chg_enable_irq(chip, CHG_GONE_IRQ);
	}

	pm8921_chg_enable_irq(chip, DCIN_VALID_IRQ);
	pm8921_chg_enable_irq(chip, USBIN_VALID_IRQ);
	pm8921_chg_enable_irq(chip, BATT_REMOVED_IRQ);
	pm8921_chg_enable_irq(chip, BATT_INSERTED_IRQ);
	pm8921_chg_enable_irq(chip, USBIN_OV_IRQ);
	pm8921_chg_enable_irq(chip, USBIN_UV_IRQ);
	pm8921_chg_enable_irq(chip, DCIN_OV_IRQ);
	pm8921_chg_enable_irq(chip, DCIN_UV_IRQ);
	pm8921_chg_enable_irq(chip, CHGFAIL_IRQ);
	pm8921_chg_enable_irq(chip, FASTCHG_IRQ);
	pm8921_chg_enable_irq(chip, VBATDET_LOW_IRQ);
	pm8921_chg_enable_irq(chip, BAT_TEMP_OK_IRQ);

	spin_lock_irqsave(&vbus_lock, flags);

/* BEGIN : janghyun.baek@lge.com 2012-03-26 for eoc check in charger logo */
#ifdef CONFIG_LGE_PM
	if (usb_chg_current || pm_chg_get_rt_status(chip, USBIN_VALID_IRQ)) {
 		/* reissue a vbus draw call */
		if(usb_chg_current)
			__pm8921_charger_vbus_draw(usb_chg_current);
		else
			__pm8921_charger_vbus_draw(init_chg_current);
			fastchg_irq_handler(chip->pmic_chg_irq[FASTCHG_IRQ], chip);
	}
#else /* QCT_ORG */
	if (usb_chg_current) {
		/* reissue a vbus draw call */
		__pm8921_charger_vbus_draw(usb_chg_current);
		fastchg_irq_handler(chip->pmic_chg_irq[FASTCHG_IRQ], chip);
	}
#endif
/* END : janghyun.baek@lge.com 2012-03-26 for eoc check in charger logo */

	spin_unlock_irqrestore(&vbus_lock, flags);

#ifdef CONFIG_LGE_WIRELESS_CHARGER
	if(pm_chg_get_rt_status(chip,DCIN_VALID_IRQ)) {
		printk(KERN_INFO "[WIRELESS] Boot with WLC_charger, DC_IRQ=%d\n",pm_chg_get_rt_status(chip, DCIN_VALID_IRQ));
		wireless_interrupt_handler(gpio_to_irq(ACTIVE_N),chip);
		dcin_valid_irq_handler(chip->pmic_chg_irq[DCIN_VALID_IRQ], chip);
	}
#endif

	fsm_state = pm_chg_get_fsm_state(chip);
	if (is_battery_charging(fsm_state)) {
		chip->bms_notify.is_charging = 1;
		pm8921_bms_charging_began();
	}

	check_battery_valid(chip);

	pr_debug("usb = %d, dc = %d  batt = %d state=%d\n",
			chip->usb_present,
			chip->dc_present,
			get_prop_batt_present(chip),
			fsm_state);
}

struct pm_chg_irq_init_data {
	unsigned int	irq_id;
	char		*name;
	unsigned long	flags;
	irqreturn_t	(*handler)(int, void *);
};

#define CHG_IRQ(_id, _flags, _handler) \
{ \
	.irq_id		= _id, \
	.name		= #_id, \
	.flags		= _flags, \
	.handler	= _handler, \
}
struct pm_chg_irq_init_data chg_irq_data[] = {
	CHG_IRQ(USBIN_VALID_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						usbin_valid_irq_handler),
	CHG_IRQ(USBIN_OV_IRQ, IRQF_TRIGGER_RISING, usbin_ov_irq_handler),
	CHG_IRQ(BATT_INSERTED_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						batt_inserted_irq_handler),
	CHG_IRQ(VBATDET_LOW_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						vbatdet_low_irq_handler),
	CHG_IRQ(USBIN_UV_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
							usbin_uv_irq_handler),
	CHG_IRQ(VBAT_OV_IRQ, IRQF_TRIGGER_RISING, vbat_ov_irq_handler),
	CHG_IRQ(CHGWDOG_IRQ, IRQF_TRIGGER_RISING, chgwdog_irq_handler),
	CHG_IRQ(VCP_IRQ, IRQF_TRIGGER_RISING, vcp_irq_handler),
	CHG_IRQ(ATCDONE_IRQ, IRQF_TRIGGER_RISING, atcdone_irq_handler),
	CHG_IRQ(ATCFAIL_IRQ, IRQF_TRIGGER_RISING, atcfail_irq_handler),
	CHG_IRQ(CHGDONE_IRQ, IRQF_TRIGGER_RISING, chgdone_irq_handler),
	CHG_IRQ(CHGFAIL_IRQ, IRQF_TRIGGER_RISING, chgfail_irq_handler),
	CHG_IRQ(CHGSTATE_IRQ, IRQF_TRIGGER_RISING, chgstate_irq_handler),
	CHG_IRQ(LOOP_CHANGE_IRQ, IRQF_TRIGGER_RISING, loop_change_irq_handler),
	CHG_IRQ(FASTCHG_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						fastchg_irq_handler),
	CHG_IRQ(TRKLCHG_IRQ, IRQF_TRIGGER_RISING, trklchg_irq_handler),
	CHG_IRQ(BATT_REMOVED_IRQ, IRQF_TRIGGER_RISING,
						batt_removed_irq_handler),
	CHG_IRQ(BATTTEMP_HOT_IRQ, IRQF_TRIGGER_RISING,
						batttemp_hot_irq_handler),
	CHG_IRQ(CHGHOT_IRQ, IRQF_TRIGGER_RISING, chghot_irq_handler),
	CHG_IRQ(BATTTEMP_COLD_IRQ, IRQF_TRIGGER_RISING,
						batttemp_cold_irq_handler),
	CHG_IRQ(CHG_GONE_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						chg_gone_irq_handler),
	CHG_IRQ(BAT_TEMP_OK_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						bat_temp_ok_irq_handler),
	CHG_IRQ(COARSE_DET_LOW_IRQ, IRQF_TRIGGER_RISING,
						coarse_det_low_irq_handler),
	CHG_IRQ(VDD_LOOP_IRQ, IRQF_TRIGGER_RISING, vdd_loop_irq_handler),
	CHG_IRQ(VREG_OV_IRQ, IRQF_TRIGGER_RISING, vreg_ov_irq_handler),
	CHG_IRQ(VBATDET_IRQ, IRQF_TRIGGER_RISING, vbatdet_irq_handler),
	CHG_IRQ(BATFET_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						batfet_irq_handler),
	CHG_IRQ(DCIN_VALID_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						dcin_valid_irq_handler),
	CHG_IRQ(DCIN_OV_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						dcin_ov_irq_handler),
	CHG_IRQ(DCIN_UV_IRQ, IRQF_TRIGGER_RISING, dcin_uv_irq_handler),
};

static int __devinit request_irqs(struct pm8921_chg_chip *chip,
					struct platform_device *pdev)
{
	struct resource *res;
	int ret, i;

	ret = 0;
	bitmap_fill(chip->enabled_irqs, PM_CHG_MAX_INTS);

	for (i = 0; i < ARRAY_SIZE(chg_irq_data); i++) {
		res = platform_get_resource_byname(pdev, IORESOURCE_IRQ,
				chg_irq_data[i].name);
		if (res == NULL) {
			pr_err("couldn't find %s\n", chg_irq_data[i].name);
			goto err_out;
		}
		chip->pmic_chg_irq[chg_irq_data[i].irq_id] = res->start;
		ret = request_irq(res->start, chg_irq_data[i].handler,
			chg_irq_data[i].flags,
			chg_irq_data[i].name, chip);
		if (ret < 0) {
			pr_err("couldn't request %d (%s) %d\n", res->start,
					chg_irq_data[i].name, ret);
			chip->pmic_chg_irq[chg_irq_data[i].irq_id] = 0;
			goto err_out;
		}
		pm8921_chg_disable_irq(chip, chg_irq_data[i].irq_id);
	}
	return 0;

err_out:
	free_irqs(chip);
	return -EINVAL;
}

static void pm8921_chg_force_19p2mhz_clk(struct pm8921_chg_chip *chip)
{
	int err;
	u8 temp;

	temp  = 0xD1;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}

	temp  = 0xD3;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}

	temp  = 0xD1;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}

	temp  = 0xD5;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}

	udelay(183);

	temp  = 0xD1;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}

	temp  = 0xD0;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}
	udelay(32);

	temp  = 0xD1;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}

	temp  = 0xD3;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}
}

static void pm8921_chg_set_hw_clk_switching(struct pm8921_chg_chip *chip)
{
	int err;
	u8 temp;

	temp  = 0xD1;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}

	temp  = 0xD0;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}
}

#define VREF_BATT_THERM_FORCE_ON	BIT(7)
static void detect_battery_removal(struct pm8921_chg_chip *chip)
{
	u8 temp;

	pm8xxx_readb(chip->dev->parent, CHG_CNTRL, &temp);
	pr_debug("upon restart CHG_CNTRL = 0x%x\n",  temp);

	if (!(temp & VREF_BATT_THERM_FORCE_ON))
		/*
		 * batt therm force on bit is battery backed and is default 0
		 * The charger sets this bit at init time. If this bit is found
		 * 0 that means the battery was removed. Tell the bms about it
		 */
		pm8921_bms_invalidate_shutdown_soc();
}

#define ENUM_TIMER_STOP_BIT	BIT(1)
#define BOOT_DONE_BIT		BIT(6)
#define CHG_BATFET_ON_BIT	BIT(3)
#define CHG_VCP_EN		BIT(0)
#define SAFE_CURRENT_MA		1500
static int __devinit pm8921_chg_hw_init(struct pm8921_chg_chip *chip)
{
	int rc;
/* BEGIN: kidong0420.kim@lge.com 2011-10-24
 * Set initial charging current based on USB max power */
	int vdd_safe;
#ifdef CONFIG_LGE_PM
	int m;
#endif /* CONFIG_LGE_PM */
/* END: kidong0420.kim@lge.com 2011-10-24 */
/* LGE_CHANGE_S [dongju99.kim@lge.com] 2012-02-15 */
#ifdef CONFIG_LGE_PM
	unsigned int cable_type;
 	unsigned int smem_size;
#endif
/* LGE_CHANGE_E [dongju99.kim@lge.com] 2012-02-15 */

	/* forcing 19p2mhz before accessing any charger registers */
	pm8921_chg_force_19p2mhz_clk(chip);

	detect_battery_removal(chip);

	rc = pm_chg_masked_write(chip, SYS_CONFIG_2,
					BOOT_DONE_BIT, BOOT_DONE_BIT);
	if (rc) {
		pr_err("Failed to set BOOT_DONE_BIT rc=%d\n", rc);
		return rc;
	}

	vdd_safe = chip->max_voltage_mv + VDD_MAX_INCREASE_MV;

	if (vdd_safe > PM8921_CHG_VDDSAFE_MAX)
		vdd_safe = PM8921_CHG_VDDSAFE_MAX;

	rc = pm_chg_vddsafe_set(chip, vdd_safe);

	if (rc) {
		pr_err("Failed to set safe voltage to %d rc=%d\n",
						chip->max_voltage_mv, rc);
		return rc;
	}
	rc = pm_chg_vbatdet_set(chip,
				chip->max_voltage_mv
				- chip->resume_voltage_delta);
	if (rc) {
		pr_err("Failed to set vbatdet comprator voltage to %d rc=%d\n",
			chip->max_voltage_mv - chip->resume_voltage_delta, rc);
		return rc;
	}

	rc = pm_chg_vddmax_set(chip, chip->max_voltage_mv);
	if (rc) {
		pr_err("Failed to set max voltage to %d rc=%d\n",
						chip->max_voltage_mv, rc);
		return rc;
	}
	rc = pm_chg_ibatsafe_set(chip, SAFE_CURRENT_MA);
	if (rc) {
		pr_err("Failed to set max voltage to %d rc=%d\n",
						SAFE_CURRENT_MA, rc);
		return rc;
	}

	rc = pm_chg_ibatmax_set(chip, chip->max_bat_chg_current);
	if (rc) {
		pr_err("Failed to set max current to 400 rc=%d\n", rc);
		return rc;
	}

	rc = pm_chg_iterm_set(chip, chip->term_current);
	if (rc) {
		pr_err("Failed to set term current to %d rc=%d\n",
						chip->term_current, rc);
		return rc;
	}

	/* Disable the ENUM TIMER */
	rc = pm_chg_masked_write(chip, PBL_ACCESS2, ENUM_TIMER_STOP_BIT,
			ENUM_TIMER_STOP_BIT);
	if (rc) {
		pr_err("Failed to set enum timer stop rc=%d\n", rc);
		return rc;
	}

/* BEGIN: kidong0420.kim@lge.com 2011-10-24
 * Set initial charging current based on USB max power */
#ifdef CONFIG_LGE_PM
	for (m = 0; m < ARRAY_SIZE(usb_ma_table); m++) {
		if (usb_ma_table[m].usb_ma == usb_chg_current)
			break;
	}
	if (m >= ARRAY_SIZE(usb_ma_table))
		m = 1;
/* LGE_CHANGE_S [dongju99.kim@lge.com] 2012-02-15 */
#ifdef CONFIG_LGE_PM
	cable_type = *(unsigned int *) (smem_get_entry(SMEM_ID_VENDOR1, &smem_size));
	if (( cable_type == LT_CABLE_56K) || ( cable_type == LT_CABLE_130K) || ( cable_type == LT_CABLE_910K )) 
	{
		m = find_usb_ma_value(C_56K_TA_MA);
		printk(KERN_INFO "cable_type is = %d\n", cable_type );
	}
	else if( cable_type == NO_BATT_QHSUSB_CHG_PORT_DCP )
	{
		m = find_usb_ma_value(C_NONE_USB_MA);
		printk(KERN_INFO "cable_type is = %d\n", cable_type );
	}
#endif
/* LGE_CHANGE_E [dongju99.kim@lge.com] 2012-02-15 */

	rc = pm_chg_iusbmax_set(chip, m);
/* BEGIN : janghyun.baek@lge.com 2012-03-26 for eoc check in charger logo */
	init_chg_current = usb_ma_table[m].usb_ma;
/* END : janghyun.baek@lge.com 2012-03-26 for eoc check in charger logo */
#else/* QCT_ORG */
	/* init with the lowest USB current */
	rc = pm_chg_iusbmax_set(chip, 0);
#endif /* CONFIG_LGE_PM */
/* END: kidong0420.kim@lge.com 2011-10-24 */
	if (chip->safety_time != 0) {
		rc = pm_chg_tchg_max_set(chip, chip->safety_time);
		if (rc) {
			pr_err("Failed to set max time to %d minutes rc=%d\n",
							chip->safety_time, rc);
			return rc;
		}
	}

	if (chip->ttrkl_time != 0) {
		rc = pm_chg_ttrkl_max_set(chip, chip->ttrkl_time);
		if (rc) {
			pr_err("Failed to set trkl time to %d minutes rc=%d\n",
							chip->safety_time, rc);
			return rc;
		}
	}

	if (chip->vin_min != 0) {
		rc = pm_chg_vinmin_set(chip, chip->vin_min);
		if (rc) {
			pr_err("Failed to set vin min to %d mV rc=%d\n",
							chip->vin_min, rc);
			return rc;
		}
	} else {
		chip->vin_min = pm_chg_vinmin_get(chip);
	}

	rc = pm_chg_disable_wd(chip);
	if (rc) {
		pr_err("Failed to disable wd rc=%d\n", rc);
		return rc;
	}

/* BEGIN: pk.jeon@lge.com 2012-04-21
 * BTM Disable in Domestic Operator and DCM, MPCS (D1LU, D1LSK, D1LKT, L_DCM, L0) */
#if defined(CONFIG_MACH_MSM8960_D1LU) || defined(CONFIG_MACH_MSM8960_D1LSK) || defined(CONFIG_MACH_MSM8960_D1LKT) || defined(CONFIG_MACH_MSM8960_L_DCM)|| defined(CONFIG_MACH_MSM8960_L0) || defined(CONFIG_MACH_MSM8960_L1sk) || defined(CONFIG_MACH_MSM8960_L1kt) || defined(CONFIG_MACH_MSM8960_L1u)
	rc = pm_chg_masked_write(chip, CHG_CNTRL_2,
				CHG_BAT_TEMP_DIS_BIT, CHG_BAT_TEMP_DIS_BIT);
#else
	rc = pm_chg_masked_write(chip, CHG_CNTRL_2,
				CHG_BAT_TEMP_DIS_BIT, 0);
#endif
/* END : pk.jeon@lge.com 2012-04-21 */


	if (rc) {
		pr_err("Failed to enable temp control chg rc=%d\n", rc);
		return rc;
	}
	/* switch to a 3.2Mhz for the buck */
	rc = pm8xxx_writeb(chip->dev->parent, CHG_BUCK_CLOCK_CTRL, 0x15);
	if (rc) {
		pr_err("Failed to switch buck clk rc=%d\n", rc);
		return rc;
	}

	if (chip->trkl_voltage != 0) {
		rc = pm_chg_vtrkl_low_set(chip, chip->trkl_voltage);
		if (rc) {
			pr_err("Failed to set trkl voltage to %dmv  rc=%d\n",
							chip->trkl_voltage, rc);
			return rc;
		}
	}

	if (chip->weak_voltage != 0) {
		rc = pm_chg_vweak_set(chip, chip->weak_voltage);
		if (rc) {
			pr_err("Failed to set weak voltage to %dmv  rc=%d\n",
							chip->weak_voltage, rc);
			return rc;
		}
	}

	if (chip->trkl_current != 0) {
		rc = pm_chg_itrkl_set(chip, chip->trkl_current);
		if (rc) {
			pr_err("Failed to set trkl current to %dmA  rc=%d\n",
							chip->trkl_voltage, rc);
			return rc;
		}
	}

	if (chip->weak_current != 0) {
		rc = pm_chg_iweak_set(chip, chip->weak_current);
		if (rc) {
			pr_err("Failed to set weak current to %dmA  rc=%d\n",
							chip->weak_current, rc);
			return rc;
		}
	}

	rc = pm_chg_batt_cold_temp_config(chip, chip->cold_thr);
	if (rc) {
		pr_err("Failed to set cold config %d  rc=%d\n",
						chip->cold_thr, rc);
	}

	rc = pm_chg_batt_hot_temp_config(chip, chip->hot_thr);
	if (rc) {
		pr_err("Failed to set hot config %d  rc=%d\n",
						chip->hot_thr, rc);
	}

	rc = pm_chg_led_src_config(chip, chip->led_src_config);
	if (rc) {
		pr_err("Failed to set charger LED src config %d  rc=%d\n",
						chip->led_src_config, rc);
	}

	/* Workarounds for die 1.1 and 1.0 */
	if (pm8xxx_get_revision(chip->dev->parent) < PM8XXX_REVISION_8921_2p0) {
		pm8xxx_writeb(chip->dev->parent, CHG_BUCK_CTRL_TEST2, 0xF1);
		pm8xxx_writeb(chip->dev->parent, CHG_BUCK_CTRL_TEST3, 0xCE);
		pm8xxx_writeb(chip->dev->parent, CHG_BUCK_CTRL_TEST3, 0xD8);

		/* software workaround for correct battery_id detection */
		pm8xxx_writeb(chip->dev->parent, PSI_TXRX_SAMPLE_DATA_0, 0xFF);
		pm8xxx_writeb(chip->dev->parent, PSI_TXRX_SAMPLE_DATA_1, 0xFF);
		pm8xxx_writeb(chip->dev->parent, PSI_TXRX_SAMPLE_DATA_2, 0xFF);
		pm8xxx_writeb(chip->dev->parent, PSI_TXRX_SAMPLE_DATA_3, 0xFF);
		pm8xxx_writeb(chip->dev->parent, PSI_CONFIG_STATUS, 0x0D);
		udelay(100);
		pm8xxx_writeb(chip->dev->parent, PSI_CONFIG_STATUS, 0x0C);
	}

	/* Workarounds for die 3.0 */
	if (pm8xxx_get_revision(chip->dev->parent) == PM8XXX_REVISION_8921_3p0)
		pm8xxx_writeb(chip->dev->parent, CHG_BUCK_CTRL_TEST3, 0xAC);

	/* Enable isub_fine resolution AICL for PM8917 */
	if (pm8xxx_get_version(chip->dev->parent) == PM8XXX_VERSION_8917)
		chip->iusb_fine_res = true;

	pm8xxx_writeb(chip->dev->parent, CHG_BUCK_CTRL_TEST3, 0xD9);

	/* Disable EOC FSM processing */
	pm8xxx_writeb(chip->dev->parent, CHG_BUCK_CTRL_TEST3, 0x91);

	rc = pm_chg_masked_write(chip, CHG_CNTRL, VREF_BATT_THERM_FORCE_ON,
						VREF_BATT_THERM_FORCE_ON);
	if (rc)
		pr_err("Failed to Force Vref therm rc=%d\n", rc);
#if defined (CONFIG_MACH_MSM8960_VU2SK) || defined (CONFIG_MACH_MSM8960_VU2U) || defined (CONFIG_MACH_MSM8960_VU2KT) || defined (CONFIG_MACH_MSM8960_L1m)
	rc = pm_chg_charge_dis(chip, charging_disabled);
	printk("%s%d\n", __func__, charging_disabled);
#else
	rc = pm_chg_charge_dis(chip, charging_disabled);
#endif	
	if (rc) {
		pr_err("Failed to disable CHG_CHARGE_DIS bit rc=%d\n", rc);
		return rc;
	}

	rc = pm_chg_auto_enable(chip, !charging_disabled);
	if (rc) {
		pr_err("Failed to enable charging rc=%d\n", rc);
		return rc;
	}

#ifdef CONFIG_LGE_WIRELESS_CHARGER
	rc = wireless_hw_init(chip);
	if (rc) {
		printk(KERN_INFO "[WIRELESS] Failed to wireless_hw_init rc=%d\n", rc);
		return rc;
	}
#endif

	return 0;
}

static int get_rt_status(void *data, u64 * val)
{
	int i = (int)data;
	int ret;

	/* global irq number is passed in via data */
	ret = pm_chg_get_rt_status(the_chip, i);
	*val = ret;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(rt_fops, get_rt_status, NULL, "%llu\n");

static int get_fsm_status(void *data, u64 * val)
{
	u8 temp;

	temp = pm_chg_get_fsm_state(the_chip);
	*val = temp;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fsm_fops, get_fsm_status, NULL, "%llu\n");

static int get_reg_loop(void *data, u64 * val)
{
	u8 temp;

	if (!the_chip) {
		pr_err("%s called before init\n", __func__);
		return -EINVAL;
	}
	temp = pm_chg_get_regulation_loop(the_chip);
	*val = temp;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(reg_loop_fops, get_reg_loop, NULL, "0x%02llx\n");

static int get_reg(void *data, u64 * val)
{
	int addr = (int)data;
	int ret;
	u8 temp;

	ret = pm8xxx_readb(the_chip->dev->parent, addr, &temp);
	if (ret) {
		pr_err("pm8xxx_readb to %x value =%d errored = %d\n",
			addr, temp, ret);
		return -EAGAIN;
	}
	*val = temp;
	return 0;
}

static int set_reg(void *data, u64 val)
{
	int addr = (int)data;
	int ret;
	u8 temp;

	temp = (u8) val;
	ret = pm8xxx_writeb(the_chip->dev->parent, addr, temp);
	if (ret) {
		pr_err("pm8xxx_writeb to %x value =%d errored = %d\n",
			addr, temp, ret);
		return -EAGAIN;
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(reg_fops, get_reg, set_reg, "0x%02llx\n");

enum {
	BAT_WARM_ZONE,
	BAT_COOL_ZONE,
};
static int get_warm_cool(void *data, u64 * val)
{
	if (!the_chip) {
		pr_err("%s called before init\n", __func__);
		return -EINVAL;
	}
	if ((int)data == BAT_WARM_ZONE)
		*val = the_chip->is_bat_warm;
	if ((int)data == BAT_COOL_ZONE)
		*val = the_chip->is_bat_cool;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(warm_cool_fops, get_warm_cool, NULL, "0x%lld\n");

static void create_debugfs_entries(struct pm8921_chg_chip *chip)
{
	int i;

	chip->dent = debugfs_create_dir("pm8921_chg", NULL);

	if (IS_ERR(chip->dent)) {
		pr_err("pmic charger couldnt create debugfs dir\n");
		return;
	}

	debugfs_create_file("CHG_CNTRL", 0644, chip->dent,
			    (void *)CHG_CNTRL, &reg_fops);
	debugfs_create_file("CHG_CNTRL_2", 0644, chip->dent,
			    (void *)CHG_CNTRL_2, &reg_fops);
	debugfs_create_file("CHG_CNTRL_3", 0644, chip->dent,
			    (void *)CHG_CNTRL_3, &reg_fops);
	debugfs_create_file("PBL_ACCESS1", 0644, chip->dent,
			    (void *)PBL_ACCESS1, &reg_fops);
	debugfs_create_file("PBL_ACCESS2", 0644, chip->dent,
			    (void *)PBL_ACCESS2, &reg_fops);
	debugfs_create_file("SYS_CONFIG_1", 0644, chip->dent,
			    (void *)SYS_CONFIG_1, &reg_fops);
	debugfs_create_file("SYS_CONFIG_2", 0644, chip->dent,
			    (void *)SYS_CONFIG_2, &reg_fops);
	debugfs_create_file("CHG_VDD_MAX", 0644, chip->dent,
			    (void *)CHG_VDD_MAX, &reg_fops);
	debugfs_create_file("CHG_VDD_SAFE", 0644, chip->dent,
			    (void *)CHG_VDD_SAFE, &reg_fops);
	debugfs_create_file("CHG_VBAT_DET", 0644, chip->dent,
			    (void *)CHG_VBAT_DET, &reg_fops);
	debugfs_create_file("CHG_IBAT_MAX", 0644, chip->dent,
			    (void *)CHG_IBAT_MAX, &reg_fops);
	debugfs_create_file("CHG_IBAT_SAFE", 0644, chip->dent,
			    (void *)CHG_IBAT_SAFE, &reg_fops);
	debugfs_create_file("CHG_VIN_MIN", 0644, chip->dent,
			    (void *)CHG_VIN_MIN, &reg_fops);
	debugfs_create_file("CHG_VTRICKLE", 0644, chip->dent,
			    (void *)CHG_VTRICKLE, &reg_fops);
	debugfs_create_file("CHG_ITRICKLE", 0644, chip->dent,
			    (void *)CHG_ITRICKLE, &reg_fops);
	debugfs_create_file("CHG_ITERM", 0644, chip->dent,
			    (void *)CHG_ITERM, &reg_fops);
	debugfs_create_file("CHG_TCHG_MAX", 0644, chip->dent,
			    (void *)CHG_TCHG_MAX, &reg_fops);
	debugfs_create_file("CHG_TWDOG", 0644, chip->dent,
			    (void *)CHG_TWDOG, &reg_fops);
	debugfs_create_file("CHG_TEMP_THRESH", 0644, chip->dent,
			    (void *)CHG_TEMP_THRESH, &reg_fops);
	debugfs_create_file("CHG_COMP_OVR", 0644, chip->dent,
			    (void *)CHG_COMP_OVR, &reg_fops);
	debugfs_create_file("CHG_BUCK_CTRL_TEST1", 0644, chip->dent,
			    (void *)CHG_BUCK_CTRL_TEST1, &reg_fops);
	debugfs_create_file("CHG_BUCK_CTRL_TEST2", 0644, chip->dent,
			    (void *)CHG_BUCK_CTRL_TEST2, &reg_fops);
	debugfs_create_file("CHG_BUCK_CTRL_TEST3", 0644, chip->dent,
			    (void *)CHG_BUCK_CTRL_TEST3, &reg_fops);
	debugfs_create_file("CHG_TEST", 0644, chip->dent,
			    (void *)CHG_TEST, &reg_fops);

	debugfs_create_file("FSM_STATE", 0644, chip->dent, NULL,
			    &fsm_fops);

	debugfs_create_file("REGULATION_LOOP_CONTROL", 0644, chip->dent, NULL,
			    &reg_loop_fops);

	debugfs_create_file("BAT_WARM_ZONE", 0644, chip->dent,
				(void *)BAT_WARM_ZONE, &warm_cool_fops);
	debugfs_create_file("BAT_COOL_ZONE", 0644, chip->dent,
				(void *)BAT_COOL_ZONE, &warm_cool_fops);

	for (i = 0; i < ARRAY_SIZE(chg_irq_data); i++) {
		if (chip->pmic_chg_irq[chg_irq_data[i].irq_id])
			debugfs_create_file(chg_irq_data[i].name, 0444,
				chip->dent,
				(void *)chg_irq_data[i].irq_id,
				&rt_fops);
	}
}

/* LGE_CHANGE
* for ATCMD
* 2011-11-04, janghyun.baek@lge.com
*/

#ifdef CONFIG_MACH_LGE
extern void machine_restart(char *cmd);

int pm8921_stop_charging_for_ATCMD(void)
{

	struct pm8921_chg_chip *chip = the_chip;

	pm8921_chg_disable_irq(chip, ATCFAIL_IRQ);
	pm8921_chg_disable_irq(chip, CHGHOT_IRQ);
	pm8921_chg_disable_irq(chip, ATCDONE_IRQ);
	pm8921_chg_disable_irq(chip, FASTCHG_IRQ);
	pm8921_chg_disable_irq(chip, CHGDONE_IRQ);
	pm8921_chg_disable_irq(chip, VBATDET_IRQ);
	pm8921_chg_disable_irq(chip, VBATDET_LOW_IRQ);

	return 1;
}
int pm8921_start_charging_for_ATCMD(void)
{

	struct pm8921_chg_chip *chip = the_chip;

	pm8921_chg_enable_irq(chip, ATCFAIL_IRQ);
	pm8921_chg_enable_irq(chip, CHGHOT_IRQ);
	pm8921_chg_enable_irq(chip, ATCDONE_IRQ);
	pm8921_chg_enable_irq(chip, FASTCHG_IRQ);
	pm8921_chg_enable_irq(chip, CHGDONE_IRQ);
	pm8921_chg_enable_irq(chip, VBATDET_IRQ);
	pm8921_chg_enable_irq(chip, VBATDET_LOW_IRQ);

	return 1;
}


static ssize_t at_chg_status_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int fsm_state, is_charging, r;
	bool b_chg_ok = false;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	fsm_state = pm_chg_get_fsm_state(the_chip);
	is_charging = is_battery_charging(fsm_state);

	if (is_charging) {
		b_chg_ok = true;
		r = sprintf(buf, "%d\n", b_chg_ok);
		printk(KERN_INFO "at_chg_status_show , true ! buf = %s, is_charging = %d, fsm=%d\n",
							buf, is_charging, fsm_state);
	} else {
		b_chg_ok = false;
		r = sprintf(buf, "%d\n", b_chg_ok);
		printk(KERN_INFO "at_chg_status_show , false ! buf = %s, is_charging = %d, fsm=%d\n",
							buf, is_charging, fsm_state);
	}

	return r;
}

static ssize_t at_chg_status_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret = 0, batt_status = 0;
	struct pm8921_chg_chip *chip = the_chip;
	/* 120307 mansu.lee@lge.com Implement Power test SOC quickstart */
#ifdef CONFIG_LGE_PM
	lge_power_test_flag = 1;
#endif
	/* 120307 mansu.lee@lge.com */

	if (!count)
		return -EINVAL;

	batt_status = get_prop_batt_status(chip);

	if (strncmp(buf, "0", 1) == 0) {
		/* stop charging */
		printk(KERN_INFO "at_chg_status_store : stop charging start\n");
		if (batt_status == POWER_SUPPLY_STATUS_CHARGING) {
			ret = pm8921_stop_charging_for_ATCMD();
			pm_chg_auto_enable(chip, 0);
			pm_chg_charge_dis(chip,1);
			printk(KERN_INFO "at_chg_status_store : stop charging end\n");
		}
	} else if (strncmp(buf, "1", 1) == 0) {
		/* start charging */
		printk(KERN_INFO "at_chg_status_store : start charging start\n");
		if (batt_status != POWER_SUPPLY_STATUS_CHARGING) {
			ret = pm8921_start_charging_for_ATCMD();
			pm_chg_auto_enable(chip, 1);
			pm_chg_charge_dis(chip,0);
			printk(KERN_INFO "at_chg_status_store : start charging end\n");
		}
	}

	if(ret == 0)
		return -EINVAL;

	return ret;
}

static ssize_t at_chg_complete_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int guage_level = 0, r = 0;
#ifdef CONFIG_BATTERY_MAX17043
    int max17043_get_soc(void);
	// After updating SOC, charging complete should be checked. 
	// 2012.09.04. MS870 PyeongTaek Production Issue by lee.yonggu@lge.com.
	//guage_level = max17043_get_capacity();
	guage_level = max17043_get_soc();
#else
	guage_level = get_prop_batt_capacity(the_chip);
#endif

	if (guage_level == 100) {
		r = sprintf(buf, "%d\n", 0);
	} else {
		r = sprintf(buf, "%d\n", 1);
	}
	return r;

}

static ssize_t at_chg_complete_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret = 0, batt_status = 0;
	struct pm8921_chg_chip *chip = the_chip;

	if (!count)
		return -EINVAL;

	batt_status = get_prop_batt_status(chip);

	if (strncmp(buf, "0", 1) == 0) {
		/* charging not complete */
		printk(KERN_INFO "at_chg_complete_store : charging not complete start\n");
		if (batt_status != POWER_SUPPLY_STATUS_CHARGING) {
			ret = pm8921_start_charging_for_ATCMD();
			pm_chg_auto_enable(chip, 1);
			pm_chg_charge_dis(chip,0);
			printk(KERN_INFO "at_chg_complete_store : charging not complete end\n");
		}
	} else if (strncmp(buf, "1", 1) == 0) {
		/* charging complete */
		printk(KERN_INFO "at_chg_complete_store : charging complete start\n");
		if (batt_status != POWER_SUPPLY_STATUS_FULL) {
			ret = pm8921_stop_charging_for_ATCMD();
			pm_chg_auto_enable(chip, 0);
			pm_chg_charge_dis(chip,1);
			printk(KERN_INFO "at_chg_complete_store : charging complete end\n");
		}
	}

	if(ret == 0)
		return -EINVAL;

	return ret;
}

static ssize_t at_pmic_reset_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int r = 0;
	bool pm_reset = true;

	msleep(3000); /* for waiting return values of testmode */

	machine_restart(NULL);

	r = sprintf(buf, "%d\n", pm_reset);

	return r;
}

/* test code by sungsookim */
static ssize_t at_current_limit_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	int r = 0;
	bool current_limit = true;

	printk(KERN_INFO "Current limit is successfully called\n");

	__pm8921_charger_vbus_draw(1500);
	r = sprintf(buf, "%d\n", current_limit);

	return r;
}

static ssize_t at_vcoin_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int r;
	struct pm8xxx_adc_chan_result result = { 0 };

	r = pm8xxx_adc_read(CHANNEL_VCOIN, &result);

	r = sprintf(buf, "%lld\n", result.physical);

	return r;
}

static ssize_t at_battemp_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct pm8xxx_adc_chan_result result;

	ret = pm8xxx_adc_read(CHANNEL_BATT_THERM, &result);
	if (ret)
	{
		pr_err("error reading adc channel = %d, ret = %d\n",
		CHANNEL_BATT_THERM, ret);
		return ret;
	}

	return sprintf(buf, "%lld\n", result.physical);
}

DEVICE_ATTR(at_current, 0644, at_current_limit_show, NULL);
DEVICE_ATTR(at_charge, 0644, at_chg_status_show, at_chg_status_store);
DEVICE_ATTR(at_chcomp, 0644, at_chg_complete_show, at_chg_complete_store);
DEVICE_ATTR(at_pmrst, 0644, at_pmic_reset_show, NULL);
DEVICE_ATTR(at_vcoin, 0644, at_vcoin_show, NULL);
DEVICE_ATTR(at_battemp, 0644, at_battemp_show, NULL);
#endif

#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
static ssize_t pm8921_batt_therm_adc_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int rc;
	struct pm8xxx_adc_chan_result result;

	rc = pm8xxx_adc_read(CHANNEL_BATT_THERM, &result);
	if (rc) {
		pr_err("error reading adc channel = %d, rc = %d\n",
					CHANNEL_BATT_THERM, rc);
		return rc;
	}
	pr_debug("batt_temp phy = %lld meas = 0x%llx adc_code = %d\n",
				result.physical, result.measurement, result.adc_code);
	rc = sprintf(buf, "%d\n", result.adc_code);
	return rc;
}
DEVICE_ATTR(batt_temp_adc, 0644, pm8921_batt_therm_adc_show, NULL);
#endif

static int pm8921_charger_suspend_noirq(struct device *dev)
{
	int rc;
	struct pm8921_chg_chip *chip = dev_get_drvdata(dev);

	rc = pm_chg_masked_write(chip, CHG_CNTRL, VREF_BATT_THERM_FORCE_ON, 0);
	if (rc)
		pr_err("Failed to Force Vref therm off rc=%d\n", rc);
	pm8921_chg_set_hw_clk_switching(chip);
	return 0;
}

static int pm8921_charger_resume_noirq(struct device *dev)
{
	int rc;
	struct pm8921_chg_chip *chip = dev_get_drvdata(dev);

	pm8921_chg_force_19p2mhz_clk(chip);

	rc = pm_chg_masked_write(chip, CHG_CNTRL, VREF_BATT_THERM_FORCE_ON,
						VREF_BATT_THERM_FORCE_ON);
	if (rc)
		pr_err("Failed to Force Vref therm on rc=%d\n", rc);
	return 0;
}

#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
static bool pm_is_chg_charge_dis_bit_set(struct pm8921_chg_chip *chip)
{
	u8 temp = 0;

	pm8xxx_readb(chip->dev->parent, CHG_CNTRL, &temp);
	return !!(temp & CHG_CHARGE_DIS_BIT);
}

static void monitor_batt_temp(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct pm8921_chg_chip *chip = container_of(dwork,
				struct pm8921_chg_chip, monitor_batt_temp_work);
	int batt_temp;
	int batt_volt;
	int stop_charging = 0;
	int ret = 0;
#if defined (CONFIG_MACH_MSM8960_VU2SK) || defined (CONFIG_MACH_MSM8960_VU2U) || defined (CONFIG_MACH_MSM8960_VU2KT) || defined (CONFIG_MACH_MSM8960_L1m)
	int dis_bit = 0;
#endif
	/* 120406 mansu.lee@lge.com Implements Thermal Mitigation iUSB setting */
	int ma = 0;
	struct pm8xxx_adc_chan_result xo_therm;
	int iusb_pre = 0;
#ifdef CONFIG_MACH_MSM8960_L1A
	int mitigation_current = usb_ma_table[1].usb_ma;
#else
	int mitigation_current = usb_ma_table[2].usb_ma;  // setting 500mA
#endif
	int xo_mitigation = 0;
	/* 120406 mansu.lee@lge.com */

	if(pseudo_batt_info.mode == 1)
		batt_temp = pseudo_batt_info.temp;
	else if (is_factory_cable())
		batt_temp = 25;
	else
		batt_temp = get_prop_batt_temp(chip) / 10;

/* LGE_CHANGE
 * fake battery icon display
 * 2012-05-10, hiro.kwon@lge.com
*/
	g_batt_temp = batt_temp;
#ifdef CONFIG_BATTERY_MAX17043
	batt_volt = max17043_get_voltage();
#else
	batt_volt = get_prop_battery_uvolts(chip) / 1000;
#endif
	stop_charging = chg_is_battery_too_hot_or_too_cold(chip, batt_temp, batt_volt);
	if(wake_lock_active(&chip->monitor_batt_temp_wake_lock) && !is_usb_chg_plugged_in(chip) && !is_dc_chg_plugged_in(chip)) {
		printk(KERN_INFO "[PM] monitor_batt_temp: Release wake lock , no charger\n");
		wake_unlock(&chip->monitor_batt_temp_wake_lock);
	}
	if (stop_charging == 1 && last_stop_charging == 0) {
		if(wake_lock_active(&chip->eoc_wake_lock)) {
			printk(KERN_INFO "[PM] monitor_batt_temp: Charging stop & wake lock by Temperature Scenario\n");
			wake_lock(&chip->monitor_batt_temp_wake_lock);
		}
		last_stop_charging = stop_charging;
		#ifdef CONFIG_LGE_WIRELESS_CHARGER
		if(is_dc_chg_plugged_in(chip)) {
			printk(KERN_INFO "[WIRELESS] Disabled by Temperature Scenario !\n");
		pm_chg_charge_dis(chip,1);
		}
		#endif
 		#if defined (CONFIG_MACH_MSM8960_VU2SK) || defined (CONFIG_MACH_MSM8960_VU2U) || defined (CONFIG_MACH_MSM8960_VU2KT)  || defined (CONFIG_MACH_MSM8960_L1m)
		dis_bit = pm_is_chg_charge_dis_bit_set(chip);
		if( ( is_usb_chg_plugged_in(chip) ) && ( 1 == dis_bit ) )
		{
		pm_chg_charge_dis(chip,0);
		printk(KERN_INFO "[dongju99.kim] dis bit is enabled by charging stop!\n");
		}
		#endif
		pm_chg_auto_enable(chip, 0);
	}
	else if (stop_charging == 0 && last_stop_charging == 1) {
		last_stop_charging = stop_charging;
		pseudo_ui_charging = 0;
		pm_chg_auto_enable(chip, 1);
		#ifdef CONFIG_LGE_WIRELESS_CHARGER
		if(is_dc_chg_plugged_in(chip)) {
			printk(KERN_INFO "[WIRELESS] Enabled by Temperature Scenario !\n");
			pm_chg_charge_dis(chip,0);
			if(!wake_lock_active(&chip->eoc_wake_lock))
				wake_lock(&chip->eoc_wake_lock);
			if(!delayed_work_pending(&chip->eoc_work))
				schedule_delayed_work(&chip->eoc_work,round_jiffies_relative(msecs_to_jiffies(WIRELESS_EARLY_EOC_WORK_TIME)));
		}
		#endif
		if(wake_lock_active(&chip->monitor_batt_temp_wake_lock)) {
			printk(KERN_INFO "[PM] monitor_batt_temp: Release wake lock by Temperature Scenario\n");
			wake_unlock(&chip->monitor_batt_temp_wake_lock);
		}
	}

	if(chip->thermal_mitigation_method == IUSB_REDUCE_METHOD)
	{
		/*
		* 120406 mansu.lee@lge.com Implements Thermal Mitigation iUSB setting.
		*
		* chager can get thermal_mitigation parameter from thermal daemon.
		* so, we decide to use this parameter for charging current mitigation.
		* if thermal daemon triggered by special threshold and do "battery" action,
		* "battery actioninfo" value is come to charger.
		* and than, it is used xo_therm compare and current mitigation value.
		*/
		pm8xxx_adc_read(CHANNEL_MUXOFF,&xo_therm);
		pr_debug("[Thermal] xo_therm = %lld , thermal_mitigation = %d\n",
			xo_therm.physical, thermal_mitigation);

		if(xo_therm.physical > (thermal_mitigation * 1000) )
			xo_mitigation = 1;
		else
			xo_mitigation = 0;

		pr_debug("[Thermal] last_stop_charging = %d. chg_state = %d. ac usb type = %d\n",
			last_stop_charging, chg_batt_temp_state, ac_online);

		if ((!last_stop_charging) && (chg_batt_temp_state != CHG_BATT_DC_CURRENT_STATE)) {
			if(ac_online){
				if ((xo_mitigation == 1) && (xo_mitigation != pre_xo_mitigation)){
					pm_chg_iusbmax_get(chip, &iusb_pre);
					pm8921_charger_vbus_draw_internal(mitigation_current); /* set 700mA*/
					usb_target_ma = mitigation_current; // to prevent to revert before target current in unplug_check_worker. 2012.10.11 by lee.yonggu@lge.com
					
					pr_err("[Thermal] resetting current iusb_pre = %d : mitigation current = %d\n",
						iusb_pre, mitigation_current);

					pre_xo_mitigation = xo_mitigation;
				}else if ((xo_mitigation == 0) && (xo_mitigation != pre_xo_mitigation)){
					ma = lge_pm_get_ta_current();
					pm_chg_iusbmax_get(chip, &iusb_pre);
					pm8921_charger_vbus_draw_internal(ma); /* set max iusb current*/
					usb_target_ma = ma;                // to prevent to revert before target current in unplug_check_worker. 2012.10.11 by lee.yonggu@lge.com
					pr_err("[Thermal] resetting current iusb_pre = %d : retore current = %d\n",
						iusb_pre, ma);

					pre_xo_mitigation = xo_mitigation;
				}
				else{
					pr_debug("[Thermal] no mitigation : current mitigation = %d / pre= %d\n",
						xo_mitigation, pre_xo_mitigation);
				}
			}
			else {
				#ifdef CONFIG_LGE_WIRELESS_CHARGER
				/* 120412 mansu.lee@lge.com Replace wireless ibat mitigation code */
				if(is_dc_chg_plugged_in(chip)) {
					pr_debug("[Thermal] Wireless charger is enabled.\n");
					if ((xo_mitigation == 1) && (xo_mitigation != pre_xo_mitigation)){
						printk(KERN_INFO "[WIRELESS] Thermal Mitigation:Decrease IBAT MAX to %dmA\n",WIRELESS_DECREASE_IBAT);
						pre_xo_mitigation = xo_mitigation;
						wireless_current_set(chip);
					}else if ((xo_mitigation == 0) && (xo_mitigation != pre_xo_mitigation)){
						printk(KERN_INFO "[WIRELESS] Thermal Mitigation:Normal IBAT MAX to %dmA\n",WIRELESS_IBAT_MAX);
						pre_xo_mitigation = xo_mitigation;
						wireless_current_set(chip);
					}
					else{
						printk(KERN_INFO "[WIRELESS] Thermal Mitigation:current mitigation = %d / pre= %d\n", xo_mitigation, pre_xo_mitigation);
					}
				}
				/* 120412 mansu.lee@lge.com */
				#endif
			}
		}
		/* 120406 mansu.lee@lge.com */
	}
	power_supply_changed(&chip->batt_psy);

	ret = schedule_delayed_work(&chip->monitor_batt_temp_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (60000)));

}
#endif

static int wireless_inform_enable = 0;
module_param(wireless_inform_enable, int, 0644);
#ifdef CONFIG_LGE_WIRELESS_CHARGER

static int wireless_ibat_max_param = WIRELESS_IBAT_MAX;
static int set_wireless_ibat_max_param(const char *val, struct kernel_param *kp)
{
	int ret;
	ret = param_set_int(val, kp);
	if (ret) {
		pr_info("error setting value %d\n", ret);
		return ret;
	}

	printk(KERN_INFO "[WIRELESS] your input : wireless_ibat_max=%d\n",wireless_ibat_max_param);
	if(!the_chip) {
		printk(KERN_INFO "[WIRELESS] Failed to IBAT_MAX\n");
	}
	else {
		pm_chg_ibatmax_set(the_chip,wireless_ibat_max_param);
	}
	return 0;
}
module_param_call(wireless_ibat_max_param,set_wireless_ibat_max_param, param_get_uint,&wireless_ibat_max_param, 0644);


static int wireless_vin_min_param = WIRELESS_VIN_MIN;
static int set_wireless_vin_min_param(const char *val, struct kernel_param *kp)
{
	int ret;
	ret = param_set_int(val, kp);
	if (ret) {
		pr_info("error setting value %d\n", ret);
		return ret;
	}

	printk(KERN_INFO "[WIRELESS] your input : wireless_vin_min_param=%d\n",wireless_vin_min_param);
	if(!the_chip) {
		printk(KERN_INFO "[WIRELESS] Failed to VIN_MIN\n");
	}
	else {
		pm_chg_vinmin_set(the_chip,wireless_vin_min_param);
	}
	return 0;
}
module_param_call(wireless_vin_min_param,set_wireless_vin_min_param, param_get_uint,&wireless_vin_min_param, 0644);


static int wireless_rx_off_param = 0;
static int wireless_rx_off_param_call(const char *val, struct kernel_param *kp)
{
	int ret;
	ret = param_set_int(val, kp);
	if (ret) {
		pr_info("error setting value %d\n", ret);
		return ret;
	}

	printk(KERN_INFO "[WIRELESS] your input : wireless_rx_off_param=%d\n",wireless_rx_off_param);
	if(!the_chip) {
		printk(KERN_INFO "[WIRELESS] Failed to wireless_rx_off_param\n");
	}
	else {
		wireless_chip_control_worker(&the_chip->wireless_chip_control_work.work);
	}
	return 0;
}
module_param_call(wireless_rx_off_param,wireless_rx_off_param_call, param_get_uint,&wireless_rx_off_param, 0644);


static int wireless_dis_param = 0;
static int wireless_dis_param_call(const char *val, struct kernel_param *kp)
{
	int ret;
	ret = param_set_int(val, kp);
	if (ret) {
		pr_info("error setting value %d\n", ret);
		return ret;
	}

	printk(KERN_INFO "[WIRELESS] your input : wireless_dis_param=%d\n",wireless_dis_param);
	if(!the_chip) {
		printk(KERN_INFO "[WIRELESS] Failed to wireless_dis_param\n");
		return 0;
	}
	else {
		pm_chg_charge_dis(the_chip,wireless_dis_param);
#if defined (CONFIG_MACH_MSM8960_VU2SK) || defined (CONFIG_MACH_MSM8960_VU2U) || defined (CONFIG_MACH_MSM8960_VU2KT) || defined (CONFIG_MACH_MSM8960_L1m)
 		printk("%s%d\n", __func__, wireless_dis_param);
#endif	
	}
	return 0;
}
module_param_call(wireless_dis_param,wireless_dis_param_call, param_get_uint,&wireless_dis_param, 0644);


static int wireless_auto_param = 0;
static int wireless_auto_param_call(const char *val, struct kernel_param *kp)
{
	int ret;
	ret = param_set_int(val, kp);
	if (ret) {
		pr_info("error setting value %d\n", ret);
		return ret;
	}

	printk(KERN_INFO "[WIRELESS] your input : wireless_auto_param=%d\n",wireless_auto_param);
	if(!the_chip) {
		printk(KERN_INFO "[WIRELESS] Failed to wireless_dis_param\n");
		return 0;
	}
	else {
		pm_chg_auto_enable(the_chip,wireless_auto_param);
	}
	return 0;
}
module_param_call(wireless_auto_param,wireless_auto_param_call, param_get_uint,&wireless_auto_param, 0644);


#ifdef CONFIG_LGE_BACKLIGHT_LM3533
static bool wireless_get_backlight_on(void)
{
	if(wireless_backlight_state()) {
		printk(KERN_INFO "[WIRELESS] ON-BackLight\n");
		return true;
	}
	else {
		printk(KERN_INFO "[WIRELESS] OFF-BackLight\n");
		return false;
	}
}
#endif
static bool wireless_is_plugged(void)
{
	int ret;
	ret = !gpio_get_value_cansleep(ACTIVE_N);
	return ret;
}


static int wireless_pm_power_get_property(struct power_supply *psy)
{
	#if 0
	int ret;

	if(wireless_charging) {
		if(the_chip)
			ret = is_dc_chg_plugged_in(the_chip);
		else
			ret = wireless_is_plugged();
	}
	return ret;
	#endif
	if(!is_dc_chg_plugged_in(the_chip) || !wireless_is_plugged()) {
		return 0;
	}
	else {
		return wireless_charging;
	}
}


static int wireless_batt_status(void)
{
	if (wireless_charge_done)
		return POWER_SUPPLY_STATUS_FULL;
	if (wireless_charging) {
		#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
		if(chg_batt_temp_state == CHG_BATT_STOP_CHARGING_STATE)// && !pseudo_ui_charging)
			return POWER_SUPPLY_STATUS_NOT_CHARGING;
		else
		#endif
			return POWER_SUPPLY_STATUS_CHARGING;
	}
	return  POWER_SUPPLY_STATUS_DISCHARGING;
}


static int wireless_soc(struct pm8921_chg_chip *chip)
{
	int soc;
	#ifdef CONFIG_BATTERY_MAX17043
	soc = max17043_get_capacity();
	#else
	soc = get_prop_batt_capacity(chip);
	#endif
	return soc;
}


static bool wireless_soc_check(struct pm8921_chg_chip *chip)
{
	int soc;
	soc = wireless_soc(chip);
	#ifdef CONFIG_BATTERY_MAX17043
	if(soc > WIRELESS_MAXIM_SOC)
	#else
	if(soc > WIRELESS_BMS_SOC)
	#endif
		return false;
	else
		return true;
}


static void turn_on_dc_ovp_fet(struct pm8921_chg_chip *chip)
{
	u8 temp;
	int rc;

	rc = pm8xxx_writeb(chip->dev->parent, DC_OVP_TEST, 0x30);
	if (rc) {
		printk(KERN_INFO "[WIRELESS] Failed to write 0x30 to DC_OVP_TEST rc = %d\n", rc);
		return;
	}
	rc = pm8xxx_readb(chip->dev->parent, DC_OVP_TEST, &temp);
	if (rc) {
		printk(KERN_INFO "[WIRELESS] Failed to read from DC_OVP_TEST rc = %d\n", rc);
		return;
	}
	/* unset ovp fet disable bit and set the write bit */
	temp &= 0xFE;
	temp |= 0x80;
	rc = pm8xxx_writeb(chip->dev->parent, DC_OVP_TEST, temp);
	if (rc) {
		printk(KERN_INFO "[WIRELESS] Failed to write 0x%x to DC_OVP_TEST rc = %d\n",temp, rc);
		return;
	}
}


static void turn_off_dc_ovp_fet(struct pm8921_chg_chip *chip)
{
	u8 temp;
	int rc;

	rc = pm8xxx_writeb(chip->dev->parent, DC_OVP_TEST, 0x30);
	if (rc) {
		printk(KERN_INFO "[WIRELESS] Failed to write 0x30 to DC_OVP_TEST rc = %d\n", rc);
		return;
	}
	rc = pm8xxx_readb(chip->dev->parent, DC_OVP_TEST, &temp);
	if (rc) {
		printk(KERN_INFO "[WIRELESS] Failed to read from DC_OVP_TEST rc = %d\n", rc);
		return;
	}
	/* set ovp fet disable bit and the write bit */
	temp |= 0x81;
	rc = pm8xxx_writeb(chip->dev->parent, DC_OVP_TEST, temp);
	if (rc) {
		printk(KERN_INFO "[WIRELESS] Failed to write 0x%x DC_OVP_TEST rc=%d\n", temp, rc);
		return;
	}
}


#define DC_OVP_DEBOUNCE_TIME 0x06
static void wireless_unplug_ovp_fet_open(struct pm8921_chg_chip *chip)
{
	int chg,dc,count;

	while (count++ < param_open_ovp_counter) {
		pm_chg_masked_write(chip,DC_OVP_CONTROL,DC_OVP_DEBOUNCE_TIME,0x0);
		usleep(10);
		dc = is_dc_chg_plugged_in(chip);
		chg = wireless_is_plugged();
		printk(KERN_INFO "[WIRELESS] OVP FET count=%d , DC_Physical=%d , DC_IRQ=%d\n",count,chg,dc);
		if (!chg && dc) {
			turn_off_dc_ovp_fet(chip);
			msleep(20);
			turn_on_dc_ovp_fet(chip);
		}
		else {
			pm_chg_masked_write(chip,DC_OVP_CONTROL,DC_OVP_DEBOUNCE_TIME,0x1);
			printk(KERN_INFO "[WIRELESS] Exit count=%d , DC_Physical=%d , DC_IRQ=%d\n",count,chg,dc);
			return;
		}
	}
}


static void wireless_current_set(struct pm8921_chg_chip *chip)
{
	#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
	if(chg_batt_temp_state == CHG_BATT_DC_CURRENT_STATE || pre_xo_mitigation) {
		pm_chg_ibatmax_set(chip,WIRELESS_DECREASE_IBAT);
		printk(KERN_INFO "[WIRELESS] SET : Decrease IBAT %d\n",WIRELESS_DECREASE_IBAT);
		return;
	}
	#endif
#ifdef CONFIG_LGE_BACKLIGHT_LM3533
	if(wireless_get_backlight_on()) {
		pm_chg_ibatmax_set(chip,WIRELESS_DECREASE_IBAT);
		printk(KERN_INFO "[WIRELESS] SET : Backlight ON, Decrease IBAT %d\n",WIRELESS_DECREASE_IBAT);
		return;
	}
#endif
	if(wireless_ibat_max_param != WIRELESS_IBAT_MAX) {
		pm_chg_ibatmax_set(chip,wireless_ibat_max_param);
		printk(KERN_INFO "[WIRELESS] SET : Param IBAT %d\n",wireless_ibat_max_param);
		return;
	}
	else {
		pm_chg_ibatmax_set(chip,WIRELESS_IBAT_MAX);
		printk(KERN_INFO "[WIRELESS] SET : Default IBAT %d\n",WIRELESS_IBAT_MAX);
		return;
	}
}


static void wireless_vinmin_set(struct pm8921_chg_chip *chip)
{
	if(wireless_vin_min_param != WIRELESS_VIN_MIN) {
		pm_chg_vinmin_set(chip,wireless_vin_min_param);
		printk(KERN_INFO "[WIRELESS] SET : Param VINMIN %d\n",wireless_vin_min_param);
	}
	else {
		pm_chg_vinmin_set(chip,WIRELESS_VIN_MIN);
		printk(KERN_INFO "[WIRELESS] SET : Default VINMIN %d\n",WIRELESS_VIN_MIN);
	}
}


/*static int wireless_batfet_on(struct pm8921_chg_chip *chip, int enable)
{
	return pm_chg_masked_write(chip, CHG_CNTRL,WIRELESS_BATFET_BIT,enable ? WIRELESS_BATFET_BIT : 0);
}*/


static void wireless_set(struct pm8921_chg_chip *chip)
{
	printk(KERN_INFO "[WIRELESS] SET START\n");
	//pm_chg_charge_dis(chip,1);
	pm_chg_failed_clear(chip, 1);

	#ifdef WIRELESS_CAYMAN
	printk(KERN_INFO "[WIRELESS] CAYMAN scenario enabled!!\n");
	if(!wireless_soc_check(chip)) {
		printk(KERN_INFO "[WIRELESS] SOC check false\n");
		wireless_chip_control_worker(&chip->wireless_chip_control_work.work);
		return;
	}
	#endif

	#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
	last_stop_charging = 0;
	pseudo_ui_charging = 0;
	pre_xo_mitigation = 0;
	if(delayed_work_pending(&chip->monitor_batt_temp_work)) {
		printk(KERN_INFO "[WIRELESS] SET : Go monitor batt temp\n");
		cancel_delayed_work_sync(&chip->monitor_batt_temp_work);
		schedule_delayed_work(&chip->monitor_batt_temp_work,round_jiffies_relative(msecs_to_jiffies(5000)));
	}
	#endif

	//pm_chg_vddmax_set(chip,WIRELESS_VDD_MAX);
	pm_chg_vbatdet_set(chip,WIRELESS_VDET_LOW);
	wireless_vinmin_set(chip);
	wireless_current_set(chip);
	pm_chg_iterm_set(chip,WIRELESS_ITERM);
	//printk(KERN_INFO "[WIRELESS] SET : ITERM %d\n",WIRELESS_ITERM);
	//printk(KERN_INFO "[WIRELESS] SET : VBATDET %d\n",WIRELESS_VDET_LOW);
	//printk(KERN_INFO "[WIRELESS] SET : VDDMAX  %d\n",WIRELESS_VDD_MAX);
	pm_chg_auto_enable(chip,1);
	pm_chg_charge_dis(chip,0);

	chip->dc_present = 1;
	wireless_charging = true;
	wireless_charge_done = false;
	wireless_source_control_count = 0;
	bms_notify_check(chip);
	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->wireless_psy);
	printk(KERN_INFO "[WIRELESS] SET END\n");
	return;
}


static void wireless_reset(struct pm8921_chg_chip *chip)
{
	if(chip->dc_present) {
		printk(KERN_INFO "[WIRELESS] RESET START\n");
	        pm_chg_failed_clear(chip, 1);
		//pm_chg_vddmax_set(chip,chip->max_voltage_mv);
		pm_chg_vbatdet_set(chip,chip->max_voltage_mv-chip->resume_voltage_delta);
		pm_chg_vinmin_set(chip,chip->vin_min);
		pm_chg_ibatmax_set(chip,chip->max_bat_chg_current);
		pm_chg_iterm_set(chip,chip->term_current);
		//wireless_batfet_on(chip,0);
		pm_chg_charge_dis(chip,0);
		pm_chg_auto_enable(chip,1);
		wireless_charging = false;
		wireless_charge_done = false;
		wireless_source_control_count = 0;
		#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
		last_stop_charging = 0;
		pseudo_ui_charging = 0;
		pre_xo_mitigation = 0;
		#endif
		chip->dc_present = 0;
		bms_notify_check(chip);
		power_supply_changed(&chip->batt_psy);
		power_supply_changed(&chip->wireless_psy);
		printk(KERN_INFO "[WIRELESS] RESET END\n");
	}
}


static void wireless_reset_hw(int usb , struct pm8921_chg_chip *chip)
{
	if(usb) {
		//pm_chg_vddmax_set(chip,chip->max_voltage_mv);
		pm_chg_vbatdet_set(chip,chip->max_voltage_mv-chip->resume_voltage_delta);
		pm_chg_vinmin_set(chip,chip->vin_min);
		pm_chg_ibatmax_set(chip,chip->max_bat_chg_current);
		pm_chg_iterm_set(chip,chip->term_current);
		//wireless_batfet_on(chip,0);
		pm_chg_charge_dis(chip,0);
		pm_chg_auto_enable(chip,1);
		printk(KERN_INFO "[WIRELESS] USB Insert - VINMIN: %dmV , IBATMAX: %dmA , VDDMAX: %dmV , VBATDET: %dmV\n",
		chip->vin_min,chip->max_bat_chg_current,chip->max_voltage_mv,chip->max_voltage_mv-chip->resume_voltage_delta);
	}
}


static void wireless_unplug_check_worker(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct pm8921_chg_chip *chip = container_of(dwork,struct pm8921_chg_chip,wireless_unplug_check_work);

	if (!is_dc_chg_plugged_in(chip)) {
		printk(KERN_INFO "[WIRELESS] DC is removed,Stopping. reg_loop=%d , fsm=%d , ibat=%d\n",pm_chg_get_regulation_loop(chip),pm_chg_get_fsm_state(chip),get_prop_batt_current(chip));
		wake_unlock(&chip->wireless_unplug_wake_lock);
		return;
	}

	if(wireless_is_plugged()) {
		printk(KERN_INFO "[WIRELESS] DC_Physical=1 , Unlock wireless_unplug_check_worker\n");
		wireless_set(chip);
		wake_unlock(&chip->wireless_unplug_wake_lock);
		schedule_delayed_work(&chip->wireless_polling_dc_work,round_jiffies_relative(msecs_to_jiffies(WIRELESS_POLLING_DC_TIME)));
		return;
	}

	if (!wireless_is_plugged() && is_dc_chg_plugged_in(chip)) {
		wireless_unplug_ovp_fet_open(chip);
		schedule_delayed_work(&chip->wireless_unplug_check_work,round_jiffies_relative(msecs_to_jiffies(WIRELESS_UNPLUG_CHECK_TIME)));
	}
}


static void wireless_polling_dc_worker(struct work_struct *work)
{
	int dc,dc_gpio,ibat;
	struct delayed_work *dwork = to_delayed_work(work);
	struct pm8921_chg_chip *chip = container_of(dwork,struct pm8921_chg_chip,wireless_polling_dc_work);

	//printk(KERN_INFO "[WIRELESS] Polling\n");

	dc = is_dc_chg_plugged_in(chip);
	if(!dc) {
		printk(KERN_INFO "[WIRELESS] Polling: DC removed\n");
		return;
	}

	dc_gpio = wireless_is_plugged();
	ibat = get_prop_batt_current(chip);
	if((ibat>0) && !dc_gpio && !wake_lock_active(&chip->wireless_unplug_wake_lock)) {
		wake_lock(&chip->wireless_unplug_wake_lock);
		schedule_delayed_work(&chip->wireless_unplug_check_work,round_jiffies_relative(msecs_to_jiffies(WIRELESS_INT_TO_UNPLUG_CHECK)));
		printk(KERN_INFO "[WIRELESS] Polling: Go wireless_unplug_check_worker\n");
		disable_irq(gpio_to_irq(ACTIVE_N));
		enable_irq(gpio_to_irq(ACTIVE_N));
		return;
	}

	schedule_delayed_work(&chip->wireless_polling_dc_work,round_jiffies_relative(msecs_to_jiffies(WIRELESS_POLLING_DC_TIME)));
}


static void wireless_interrupt_worker(struct work_struct *work)
{
	struct pm8921_chg_chip *chip = container_of(work,struct pm8921_chg_chip,wireless_interrupt_work);

	printk(KERN_INFO "[WIRELESS] INTWORKER START\n");
	if(wireless_is_plugged())
		wireless_set(chip);
	else
		wireless_reset(chip);

	if(!wireless_is_plugged() && is_dc_chg_plugged_in(chip) && !delayed_work_pending(&chip->wireless_unplug_check_work)) {
		printk(KERN_INFO "[WIRELESS] INT: Do Schedule unplug check\n");
		wake_lock(&chip->wireless_unplug_wake_lock);
		schedule_delayed_work(&chip->wireless_unplug_check_work,round_jiffies_relative(msecs_to_jiffies(WIRELESS_INT_TO_UNPLUG_CHECK)));
	}
	if(wireless_is_plugged() && delayed_work_pending(&chip->wireless_unplug_check_work)) {
		if(wake_lock_active(&chip->wireless_unplug_wake_lock))
			wake_unlock(&chip->wireless_unplug_wake_lock);
		cancel_delayed_work_sync(&chip->wireless_unplug_check_work);
		printk(KERN_INFO "[WIRELESS] INT: Cancel schedule unplug check\n");
	}
	printk(KERN_INFO "[WIRELESS] INTWORKER END\n");
}


static irqreturn_t  wireless_interrupt_handler(int irq, void *data)
{
	int chg;
	struct pm8921_chg_chip *chip = data;

	printk(KERN_INFO "[WIRELESS] ACTIVE INT START===========================\n");
	chg = wireless_is_plugged();
	printk(KERN_INFO "[WIRELESS] INT DC_Physical = %d\n",chg);
	schedule_work(&chip->wireless_interrupt_work);
	printk(KERN_INFO "[WIRELESS] ACTIVE INT END=============================\n");
	return IRQ_HANDLED;
}


static void wireless_interrupt_probe(struct pm8921_chg_chip *chip)
{
	int ret;

	pm8921_chg_enable_irq(chip, BATFET_IRQ);
	pm8921_chg_enable_irq(chip, CHGSTATE_IRQ);

	ret = gpio_request(ACTIVE_N,"pm8921_wireless_int");
	if (ret < 0) {
		printk(KERN_INFO "[WIRELESS] gpio_request failed\n");
		return;
	}
	gpio_direction_input(ACTIVE_N);
	ret = request_irq(gpio_to_irq(ACTIVE_N),wireless_interrupt_handler,IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING,"WLC_ACTIVE_N",chip);
	if (ret < 0) {
		printk(KERN_INFO "[WIRELESS] request_irq failed\n");
		return;
	}
	enable_irq_wake(gpio_to_irq(ACTIVE_N));
	return;
}


static void dcin_valid_irq_worker(struct work_struct *work)
{
	struct pm8921_chg_chip *chip = container_of(work,struct pm8921_chg_chip,dcin_valid_irq_work);

	/*if(!wireless_interrupt_enable) {
		printk(KERN_INFO "[WIRELESS] Enable Active_N IRQ\n");
		enable_irq(gpio_to_irq(ACTIVE_N));
		disable_irq(gpio_to_irq(ACTIVE_N));
		enable_irq(gpio_to_irq(ACTIVE_N));
		wireless_interrupt_enable = true;
	}*/

	if(is_dc_chg_plugged_in(chip)&&!is_usb_chg_plugged_in(chip)) {
		if(delayed_work_pending(&chip->eoc_work)) {
			if(wake_lock_active(&chip->eoc_wake_lock))
				wake_unlock(&chip->eoc_wake_lock);
			cancel_delayed_work_sync(&chip->eoc_work);
			printk(KERN_INFO "[WIRELESS] DC-Cancel previous eoc_work\n");
		}
		if(!wake_lock_active(&chip->eoc_wake_lock))
			wake_lock(&chip->eoc_wake_lock);
		schedule_delayed_work(&chip->eoc_work,round_jiffies_relative(msecs_to_jiffies(WIRELESS_EARLY_EOC_WORK_TIME)));
		printk(KERN_INFO "[WIRELESS] Go eoc_worker\n");
	}

	if(delayed_work_pending(&chip->wireless_unplug_check_work)) {
		if(wake_lock_active(&chip->wireless_unplug_wake_lock))
			wake_unlock(&chip->wireless_unplug_wake_lock);
		cancel_delayed_work_sync(&chip->wireless_unplug_check_work);
		printk(KERN_INFO "[WIRELESS] Cancel wireless_unplug_work\n");
	}
	else {
		printk(KERN_INFO "[WIRELESS] Nothing to cancel wireless_unplug_work\n");
	}

	if(is_dc_chg_plugged_in(chip)) {
		printk(KERN_INFO "[WIRELESS] dc worker - SET\n");
		wireless_set(chip);
	}
	else{
		printk(KERN_INFO "[WIRELESS] dc worker - RESET\n");
		wireless_reset(chip);
	}
}


static irqreturn_t dcin_valid_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;
	int dc_present;
	/*struct timeval time;
	long gap;*/
	int dc_gpio;

	printk(KERN_INFO "[WIRELESS] DC IRQ START\n");
	dc_present = pm_chg_get_rt_status(chip, DCIN_VALID_IRQ);
	if(dc_present) {
		printk(KERN_INFO "[WIRELESS] Schedule DC Polling\n");
		schedule_delayed_work(&chip->wireless_polling_dc_work,round_jiffies_relative(msecs_to_jiffies(WIRELESS_POLLING_DC_TIME)));
	}
	/*if(dc_present && !wireless_dc_time) {
		printk(KERN_INFO "[WIRELESS] wireless_interrupt_enable\n");
		wireless_interrupt_handler(gpio_to_irq(ACTIVE_N),data);
		do_gettimeofday(&time);
		wireless_dc_time = time.tv_sec;
	}*/
	dc_gpio = wireless_is_plugged();
	printk(KERN_INFO "[WIRELESS] SAVED_DC=%d , NOW_DC=%d , GPIO=%d\n",chip->dc_present,dc_present,dc_gpio);
	schedule_work(&chip->dcin_valid_irq_work);
	/*gap = wireless_dc_time-wireless_on_time;
	if(gap > 0 && gap < 6)
		printk(KERN_INFO "[WIRELESS] ON ~ DC Abnormal triger !!!\n");*/
	printk(KERN_INFO "[WIRELESS] DC IRQ END\n");
	return IRQ_HANDLED;
}


static void wireless_source_control_worker(struct work_struct *work)
{
	struct delayed_work	*dwork = to_delayed_work(work);
	struct pm8921_chg_chip	*chip = container_of(dwork,struct pm8921_chg_chip,wireless_source_control_work);

	if(is_dc_chg_plugged_in(chip) && !is_usb_chg_plugged_in(chip)) {
		if(!wireless_source_toggle) {
			wake_lock(&chip->wireless_source_wake_lock);
			pm_chg_charge_dis(chip,1);
			//pm_chg_vddmax_set(chip,chip->max_voltage_mv);
			pm_chg_vbatdet_set(chip,chip->max_voltage_mv-chip->resume_voltage_delta);
			schedule_delayed_work(&chip->wireless_source_control_work,round_jiffies_relative(msecs_to_jiffies(WIRELESS_EARLY_SOURCE_TIME)));
			wireless_source_toggle = true;
			printk(KERN_INFO "[WIRELESS] Source Disabled. FSM=%d,SOC=%d\n",pm_chg_get_fsm_state(chip),wireless_soc(chip));

		}
		else {
			//pm_chg_vddmax_set(chip,WIRELESS_VDD_MAX);
			pm_chg_vbatdet_set(chip,WIRELESS_VDET_LOW);
			pm_chg_charge_dis(chip,0);
			if(delayed_work_pending(&chip->eoc_work)) {
				if(wake_lock_active(&chip->eoc_wake_lock))
					wake_unlock(&chip->eoc_wake_lock);
				cancel_delayed_work_sync(&chip->eoc_work);
				printk(KERN_INFO "[WIRELESS] Source-Cancel previous eoc_work\n");
			}
			if(!wake_lock_active(&chip->eoc_wake_lock))
				wake_lock(&chip->eoc_wake_lock);
			schedule_delayed_work(&chip->eoc_work,round_jiffies_relative(msecs_to_jiffies(WIRELESS_EARLY_EOC_WORK_TIME)));
			wireless_source_toggle = false;
			wireless_source_control_count++;
			printk(KERN_INFO "[WIRELESS] Source Enabled , go eoc_worker\n");
			wake_unlock(&chip->wireless_source_wake_lock);
		}
	}
	else {
		printk(KERN_INFO "[WIRELESS] Source-Do Nothing , USB=%d\n",is_usb_chg_plugged_in(chip));
	}
}


static void wireless_chip_control_worker(struct work_struct *work)
{
	struct delayed_work	*dwork = to_delayed_work(work);
	struct pm8921_chg_chip	*chip = container_of(dwork,struct pm8921_chg_chip,wireless_chip_control_work);
	//struct timeval time;

	if(wireless_is_plugged()) {
		wake_lock(&chip->wireless_chip_wake_lock);
		printk(KERN_INFO "[WIRELESS] Turn Off RX-Chip.  FSM=%d,SOC=%d\n",pm_chg_get_fsm_state(chip),wireless_soc(chip));
		pm_chg_charge_dis(chip,1);
		gpio_set_value_cansleep(CHG_STAT,WIRELESS_OFF);
		schedule_delayed_work(&chip->wireless_chip_control_work,round_jiffies_relative(msecs_to_jiffies(WIRELESS_CHIP_CONTROL_TIME)));
		wireless_source_control_count=0;
	}
	else{
		printk(KERN_INFO "[WIRELESS] Turn On RX-Chip\n");
		/*do_gettimeofday(&time);
		wireless_dc_time = 0;
		wireless_on_time = time.tv_sec;
		wireless_interrupt_enable = false;
		disable_irq(gpio_to_irq(ACTIVE_N));*/
		gpio_set_value_cansleep(CHG_STAT,WIRELESS_ON);
		wake_unlock(&chip->wireless_chip_wake_lock);
	}
}


static int wireless_is_charging_finished(struct pm8921_chg_chip *chip)
{
	int vbatdet_low;
	int ichg_meas_ma,iterm_programmed;
	int fast_chg, vcp;
	int soc;
	int rc;

	wireless_current_set(chip);

	if(!last_stop_charging) {
		fast_chg = pm_chg_get_rt_status(chip, FASTCHG_IRQ);
		printk(KERN_INFO "[WIRELESS] fast_chg = %d\n", fast_chg);
		if (fast_chg == 0) {
			if(pm_chg_get_fsm_state(chip) == FSM_STATE_TRKL_CHG_8) {
				printk(KERN_INFO "[WIRELESS] TRICKLE\n");
				return CHG_IN_PROGRESS;
			}

			if(wireless_soc_check(chip) && wireless_source_control_count < WIRELESS_SRC_COUNT_LIMIT) {
				printk(KERN_INFO "[WIRELESS] NOT FSM 7 !!!  Source Control. Count=%d\n",wireless_source_control_count);
				wireless_source_control_worker(&chip->wireless_source_control_work.work);

			}
			else {
				printk(KERN_INFO "[WIRELESS] NOT FSM 7 !!!  Reset TI-CHIP. Count=%d\n",wireless_source_control_count);
				wireless_chip_control_worker(&chip->wireless_chip_control_work.work);
			}
			return CHG_NOT_IN_PROGRESS;
		}
	}
	else {
		printk(KERN_INFO "[WIRELESS] Skip check fast_chg, last_stop_charing=%d\n",last_stop_charging);
		return CHG_NOT_IN_PROGRESS;
	}

	wireless_source_control_count = 0;
	vcp = pm_chg_get_rt_status(chip, VCP_IRQ);
	printk(KERN_INFO "[WIRELESS] vcp = %d\n", vcp);
	if (vcp == 1)
		return CHG_IN_PROGRESS;
	vbatdet_low = pm_chg_get_rt_status(chip, VBATDET_LOW_IRQ);
	printk(KERN_INFO "[WIRELESS] vbatdet_low = %d\n", vbatdet_low);
	//if (vbatdet_low == 1)
	//	return CHG_IN_PROGRESS;
	rc = pm_chg_get_rt_status(chip, BAT_TEMP_OK_IRQ);
	printk(KERN_INFO "[WIRELESS] batt_temp_ok = %d\n", rc);
	if (rc == 0)
		return CHG_IN_PROGRESS;
	#ifdef CONFIG_BATTERY_MAX17043
	soc = max17043_get_capacity();
	#else
	soc = get_prop_batt_capacity(chip);
	#endif
	if(soc < 100) {
		printk(KERN_INFO "[WIRELESS] SOC is %d\n",soc);
		return CHG_IN_PROGRESS;
	}
	rc = pm_chg_iterm_get(chip, &iterm_programmed);
	if (rc) {
		printk(KERN_INFO "[WIRELESS] couldnt read iterm rc = %d\n", rc);
		return CHG_IN_PROGRESS;
	}
	ichg_meas_ma = (get_prop_batt_current(chip)) / 1000;
	printk(KERN_INFO "[WIRELESS] iterm_programmed = %d ichg_meas_ma=%d\n",iterm_programmed, ichg_meas_ma);
	if (ichg_meas_ma > 0) {
		printk(KERN_INFO "[WIRELESS] IBAT is providing from BATT,still CHG_IN_PROGERESS\n");
		if(soc > 99) {
			printk(KERN_INFO "[WIRELESS] SOC is %d but IBAT is providing, CHG_FINISHED\n",soc);
			return CHG_FINISHED;
		}
		else{
			return CHG_IN_PROGRESS;
		}
	}
	if (ichg_meas_ma * -1 > iterm_programmed) {
		return CHG_IN_PROGRESS;
	}
	return CHG_FINISHED;
}


static int wireless_hw_init(struct pm8921_chg_chip *chip)
{
	int rc;

	if(wireless_is_plugged()) {
		printk(KERN_INFO "[WIRELESS] Boot & Probing by Wireless Charger \n");
		rc = pm_chg_charge_dis(chip,1);
		if (rc) {
			printk(KERN_INFO "[WIRELESS] Failed to disable CHG_CHARGE_DIS bit rc=%d\n", rc);
			return rc;
		}

		rc = pm_chg_auto_enable(chip,0);
		if (rc) {
			printk(KERN_INFO "[WIRELESS] Failed to enable charging rc=%d\n", rc);
			return rc;
		}
	}
	return rc;
}
#endif

static void wireless_information(struct work_struct *work)
{
	int ret=0;		int rc=0;
	int regloop=0;		int vin_min=0;		int ibat=0;
	bool dis=0;		u8 ato = 0;		u8 ibat_get=0;
	int this_ibat=0;	int fast=0;		int batt_temp=0;
	int maxim_volt=0;	int vbatdet_low=0;	int dc=0;
	int dc_gpio=0;		int fsm=0;		int fet=0;
	int usb=0;		int bms_volt=0;		int vbat_max=0;
	u8 temp=0;		int vdet_mv=0;		int maxim_soc=0;
	int bms_soc=0;		int iterm=0;		u8 batfet=0;

	struct pm8xxx_adc_chan_result	result;
	struct pm8xxx_adc_chan_result	xo_therm;
	struct delayed_work		*dwork = to_delayed_work(work);
	struct pm8921_chg_chip		*chip = container_of(dwork,struct pm8921_chg_chip,wireless_inform_work);


			usb	= is_usb_chg_plugged_in(the_chip);
#ifdef CONFIG_LGE_WIRELESS_CHARGER
			dc_gpio = wireless_is_plugged();
#else
            dc_gpio = 0;
#endif
			regloop	= pm_chg_get_regulation_loop(the_chip);
			vin_min = pm_chg_vinmin_get(the_chip);
			ibat 	= get_prop_batt_current(the_chip);
			fsm 	= pm_chg_get_fsm_state(the_chip);
			fet 	= pm_chg_get_rt_status(the_chip, BATFET_IRQ);
			rc 	= pm8xxx_adc_read(CHANNEL_DCIN,&result);
			dis	= pm_is_chg_charge_dis_bit_set(the_chip);
			pm8xxx_readb(the_chip->dev->parent,CHG_CNTRL_3,&ato);
			ato 	= !!(ato & CHG_EN_BIT);
			pm8xxx_readb(the_chip->dev->parent,CHG_IBAT_MAX,&ibat_get);
			ibat_get = (ibat_get&PM8921_CHG_I_MASK);
			this_ibat = (ibat_get*PM8921_CHG_I_STEP_MA)+PM8921_CHG_I_MIN_MA;
			fast 	= pm_chg_get_rt_status(the_chip, FASTCHG_IRQ);
			batt_temp = get_prop_batt_temp(the_chip) / 10;
			#ifdef CONFIG_BATTERY_MAX17043
			maxim_volt = max17043_get_voltage();
			#endif
			bms_volt = get_prop_battery_uvolts(the_chip) / 1000;
			vbatdet_low = pm_chg_get_rt_status(the_chip, VBATDET_LOW_IRQ);
			pm_chg_vddmax_get(the_chip,&vbat_max);
			pm8xxx_readb(the_chip->dev->parent, CHG_VBAT_DET, &temp);
			temp = temp&PM8921_CHG_V_MASK;
			vdet_mv = temp*PM8921_CHG_V_STEP_MV+PM8921_CHG_V_MIN_MV;
#ifdef CONFIG_BATTERY_MAX17043
			maxim_soc = max17043_get_capacity();
#endif
#if !defined(CONFIG_BATTERY_MAX17043)
			bms_soc = get_prop_batt_capacity(the_chip);
#endif
			pm_chg_iterm_get(the_chip,&iterm);
			pm8xxx_readb(the_chip->dev->parent,CHG_CNTRL,&batfet);
#ifdef CONFIG_LGE_WIRELESS_CHARGER
			batfet 	= !!(batfet & WIRELESS_BATFET_BIT);
#endif
			pm8xxx_adc_read(CHANNEL_MUXOFF,&xo_therm);

#ifdef CONFIG_LGE_WIRELESS_CHARGER
			printk(KERN_INFO "[WIRELESS]-%d >>>\n",wireless_inform_no_dc);
			printk(KERN_INFO "| wireless_charging=%d\n",wireless_charging);
#endif
			printk(KERN_INFO "| DC_IRQ=%d(GPIO=%d) , USB_IRQ=%d\n",dc,dc_gpio,usb);
			printk(KERN_INFO "| VIN_MIN=%dmV , DC_IN=%dmV\n",vin_min,((int)result.physical/1000));
			printk(KERN_INFO "| VBAT_MAX=%dmV , VBAT_LOW=%dmV\n" , vbat_max,vdet_mv);
			printk(KERN_INFO "| Maxim_Volt=%dmV , BMS_Volt=%dmV\n",maxim_volt,bms_volt);
			printk(KERN_INFO "| Maxim_SOC=%d , BMS_SOC=%d\n",maxim_soc,bms_soc);
			printk(KERN_INFO "| IBAT_MAX=%dmA , IBAT=%dmA , ITERM=%dmA\n",this_ibat,(ibat/1000),iterm);
			printk(KERN_INFO "| FET_IRQ=%d , FET_FORCE=%d , FSM=%d , Dis_Bit=%d , Auto_Enable=%d , RegLoop=%d\n",fet,batfet,fsm,dis,ato,regloop);
			printk(KERN_INFO "| Low_IRQ=%d , FAST_IRQ=%d\n",vbatdet_low,fast);
			printk(KERN_INFO "| BAT_TEMP=%d , XO_Therm=%d(%d) , XO_Mitigation=%d\n",batt_temp,((int)xo_therm.physical/1000),thermal_mitigation,pre_xo_mitigation);
			printk(KERN_INFO "| chg_batt_temp_state=%d , last_stop_charging=%d , pseudo_ui_charging=%d\n",chg_batt_temp_state,last_stop_charging,pseudo_ui_charging);
#ifdef CONFIG_LGE_WIRELESS_CHARGER
			printk(KERN_INFO "[WIRELESS]-%d <<<\n",wireless_inform_no_dc);
#endif
		ret = schedule_delayed_work(&chip->wireless_inform_work,round_jiffies_relative(msecs_to_jiffies(WIRELESS_INFORM_NORMAL_TIME)));
}


static void wireless_probe(struct pm8921_chg_chip *chip)
{
#ifdef CONFIG_LGE_WIRELESS_CHARGER
	printk(KERN_INFO "[WIRELESS] Probe\n");
	INIT_WORK(&chip->wireless_interrupt_work,wireless_interrupt_worker);
	INIT_WORK(&chip->dcin_valid_irq_work,dcin_valid_irq_worker);
	wake_lock_init(&chip->wireless_chip_wake_lock,WAKE_LOCK_SUSPEND,"pm8921_wireless_chip");
	wake_lock_init(&chip->wireless_source_wake_lock,WAKE_LOCK_SUSPEND,"pm8921_wireless_source");
	wake_lock_init(&chip->wireless_unplug_wake_lock,WAKE_LOCK_SUSPEND,"pm8921_wireless_unplug");
	INIT_DELAYED_WORK(&chip->wireless_unplug_check_work,wireless_unplug_check_worker);
	INIT_DELAYED_WORK(&chip->wireless_chip_control_work,wireless_chip_control_worker);
	INIT_DELAYED_WORK(&chip->wireless_source_control_work,wireless_source_control_worker);
	INIT_DELAYED_WORK(&chip->wireless_polling_dc_work,wireless_polling_dc_worker);
#endif

    INIT_DELAYED_WORK(&chip->wireless_inform_work,wireless_information);
	schedule_delayed_work(&chip->wireless_inform_work,round_jiffies_relative(msecs_to_jiffies(WIRELESS_INFORM_NORMAL_TIME)));

#ifdef CONFIG_LGE_WIRELESS_CHARGER	
	wireless_interrupt_probe(chip);
#endif

}



static int pm8921_charger_resume(struct device *dev)
{
	int rc;
	struct pm8921_chg_chip *chip = dev_get_drvdata(dev);

#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
	if (chip) {
		rc = schedule_delayed_work(&chip->monitor_batt_temp_work,
				  round_jiffies_relative(msecs_to_jiffies
							 (5000)));
	}
#endif
//#ifdef CONFIG_LGE_WIRELESS_CHARGER
	if(chip){
		if(wireless_inform_enable)
			rc = schedule_delayed_work(&chip->wireless_inform_work,round_jiffies_relative(msecs_to_jiffies(WIRELESS_INFORM_RAPID_TIME)));
		else
			rc = schedule_delayed_work(&chip->wireless_inform_work,round_jiffies_relative(msecs_to_jiffies(WIRELESS_INFORM_NORMAL_TIME)));
	}
//#endif

/* BEGIN : janghyun.baek@lge.com 2012-03-10 LGE devices use temp scenario, don't use BTM. */
#ifdef CONFIG_MACH_LGE
	if (!(chip->cool_temp_dc == 0 && chip->warm_temp_dc == 0)
#else /* QCT_ORG */
	if (!(chip->cool_temp_dc == INT_MIN && chip->warm_temp_dc == INT_MIN)
#endif
/* END : janghyun.baek@lge.com 2012-03-10 */
		&& !(chip->keep_btm_on_suspend)) {
		rc = pm8xxx_adc_btm_configure(&btm_config);
		if (rc)
			pr_err("couldn't reconfigure btm rc=%d\n", rc);

		rc = pm8xxx_adc_btm_start();
		if (rc)
			pr_err("couldn't restart btm rc=%d\n", rc);
	}
	if (pm8921_chg_is_enabled(chip, LOOP_CHANGE_IRQ)) {
		disable_irq_wake(chip->pmic_chg_irq[LOOP_CHANGE_IRQ]);
		pm8921_chg_disable_irq(chip, LOOP_CHANGE_IRQ);
	}
	return 0;
}

static int pm8921_charger_suspend(struct device *dev)
{
	int rc;
	struct pm8921_chg_chip *chip = dev_get_drvdata(dev);

#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
	if (chip) {
		rc = cancel_delayed_work_sync(&chip->monitor_batt_temp_work);
	}
#endif

#ifdef CONFIG_LGE_WIRELESS_CHARGER
	if (chip)
		rc = cancel_delayed_work_sync(&chip->wireless_inform_work);

#endif

/* BEGIN : janghyun.baek@lge.com 2012-03-10 LGE devices use temp scenario, don't use BTM. */
#ifdef CONFIG_MACH_LGE
	if (!(chip->cool_temp_dc == 0 && chip->warm_temp_dc == 0)
#else /* QCT_ORG */
	if (!(chip->cool_temp_dc == INT_MIN && chip->warm_temp_dc == INT_MIN)
#endif
/* END : janghyun.baek@lge.com 2012-03-10 */

		&& !(chip->keep_btm_on_suspend)) {
		rc = pm8xxx_adc_btm_end();
		if (rc)
			pr_err("Failed to disable BTM on suspend rc=%d\n", rc);
	}

/* Battery alarm for the model using BMS */
#if defined(CONFIG_LGE_PM) && !defined(CONFIG_BATTERY_MAX17043)
	if (batt_alarm_disabled == 1) {
		batt_alarm_disabled = 0;
		rc = pm8xxx_batt_alarm_enable(0);	// PM8XXX_BATT_ALARM_LOWER_COMPARATOR - 0
		if (rc)
			pr_err(	"[batt_alarm] pm8xxx_batt_alarm_enable : unable to set alarm state. rc = %d\n", rc);
		pr_info("[batt_alarm] enable the alarm.\n");
	}
#endif

#ifdef CONFIG_LGE_WIRELESS_CHARGER
	if (is_usb_chg_plugged_in(chip)||is_dc_chg_plugged_in(chip))
#else
	if (is_usb_chg_plugged_in(chip))
#endif
	{
		pm8921_chg_enable_irq(chip, LOOP_CHANGE_IRQ);
		enable_irq_wake(chip->pmic_chg_irq[LOOP_CHANGE_IRQ]);
	}

	return 0;
}
static int __devinit pm8921_charger_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct pm8921_chg_chip *chip;
	const struct pm8921_charger_platform_data *pdata
				= pdev->dev.platform_data;

	if (!pdata) {
		pr_err("missing platform data\n");
		return -EINVAL;
	}

	chip = kzalloc(sizeof(struct pm8921_chg_chip),
					GFP_KERNEL);
	if (!chip) {
		pr_err("Cannot allocate pm_chg_chip\n");
		return -ENOMEM;
	}

	chip->dev = &pdev->dev;
	chip->safety_time = pdata->safety_time;
	chip->ttrkl_time = pdata->ttrkl_time;
	chip->update_time = pdata->update_time;
/* L0, Change Battery Profile
*  Battery is changed from 4.20V to 4.35V after Rev.C
*  2012-03-19, junsin.park@lge.com
*/
#if defined(CONFIG_MACH_MSM8960_L0)
	if(lge_get_board_revno() < HW_REV_D)
		chip->max_voltage_mv = 4200;
	else
#endif
	chip->max_voltage_mv = pdata->max_voltage;
	chip->min_voltage_mv = pdata->min_voltage;
	chip->resume_voltage_delta = pdata->resume_voltage_delta;
	chip->term_current = pdata->term_current;
	chip->vbat_channel = pdata->charger_cdata.vbat_channel;
	chip->batt_temp_channel = pdata->charger_cdata.batt_temp_channel;
	chip->batt_id_channel = pdata->charger_cdata.batt_id_channel;
	chip->batt_id_min = pdata->batt_id_min;
	chip->batt_id_max = pdata->batt_id_max;
	if (pdata->cool_temp != INT_MIN)
		chip->cool_temp_dc = pdata->cool_temp * 10;
	else
		chip->cool_temp_dc = INT_MIN;

	if (pdata->warm_temp != INT_MIN)
		chip->warm_temp_dc = pdata->warm_temp * 10;
	else
		chip->warm_temp_dc = INT_MIN;

	chip->temp_check_period = pdata->temp_check_period;
	chip->max_bat_chg_current = pdata->max_bat_chg_current;
	chip->cool_bat_chg_current = pdata->cool_bat_chg_current;
	chip->warm_bat_chg_current = pdata->warm_bat_chg_current;
	chip->cool_bat_voltage = pdata->cool_bat_voltage;
	chip->warm_bat_voltage = pdata->warm_bat_voltage;
	chip->keep_btm_on_suspend = pdata->keep_btm_on_suspend;
	chip->trkl_voltage = pdata->trkl_voltage;
	chip->weak_voltage = pdata->weak_voltage;
	chip->trkl_current = pdata->trkl_current;
	chip->weak_current = pdata->weak_current;
	chip->vin_min = pdata->vin_min;
	chip->thermal_mitigation = pdata->thermal_mitigation;
	chip->thermal_levels = pdata->thermal_levels;

	chip->cold_thr = pdata->cold_thr;
	chip->hot_thr = pdata->hot_thr;
	chip->rconn_mohm = pdata->rconn_mohm;
	chip->led_src_config = pdata->led_src_config;

#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
	last_stop_charging = 0;
	chg_batt_temp_state = CHG_BATT_NORMAL_STATE;
	pseudo_ui_charging = 0;
/* 120406 mansu.lee@lge.com Implements Thermal Mitigation iUSB setting */
	pre_xo_mitigation = 0;
	thermal_mitigation = 100; /* default 100 DegC */
/* 120406 mansu.lee@lge.com */
/* BEGIN : jooyeong.lee@lge.com 2012-02-27 Change the charger_temp_scenario */
	chip->temp_level_1 = pdata->temp_level_1;
	chip->temp_level_2 = pdata->temp_level_2;
	chip->temp_level_3 = pdata->temp_level_3;
	chip->temp_level_4 = pdata->temp_level_4;
	chip->temp_level_5 = pdata->temp_level_5;
/* END : jooyeong.lee@lge.com 2012-02-27 */
/* LGE_CHANGE
 * add the xo_thermal mitigation way
 * 2012-04-10, hiro.kwon@lge.com
*/
	chip->thermal_mitigation_method = pdata->thermal_mitigation_method;
/* 2012-04-10, hiro.kwon@lge.com */
#endif
#ifdef CONFIG_MACH_LGE
	chip->batt_id_gpio	= pdata->batt_id_gpio;		/* No. of msm gpio for battery id */
	chip->batt_id_pu_gpio	= pdata->batt_id_pu_gpio;	/* No. of msm gpio for battery id pull up */
#endif
	rc = pm8921_chg_hw_init(chip);
	if (rc) {
		pr_err("couldn't init hardware rc=%d\n", rc);
		goto free_chip;
	}

	chip->usb_psy.name = "usb",
	chip->usb_psy.type = POWER_SUPPLY_TYPE_USB,
	chip->usb_psy.supplied_to = pm_power_supplied_to,
	chip->usb_psy.num_supplicants = ARRAY_SIZE(pm_power_supplied_to),
	chip->usb_psy.properties = pm_power_props_usb,
	chip->usb_psy.num_properties = ARRAY_SIZE(pm_power_props_usb),
	chip->usb_psy.get_property = pm_power_get_property_usb,
/* BEGIN : janghyun.baek@lge.com 2012-05-06 for TA/USB recognition */
#ifdef CONFIG_LGE_PM
	chip->usb_psy.set_property = pm_power_set_property_usb,
#endif
/* END : janghyun.baek@lge.com 2012-05-06 */

#ifdef CONFIG_LGE_WIRELESS_CHARGER
	chip->wireless_psy.name = "wireless",
	chip->wireless_psy.type = POWER_SUPPLY_TYPE_WIRELESS,
	chip->wireless_psy.supplied_to = pm_power_supplied_to,
	chip->wireless_psy.num_supplicants = ARRAY_SIZE(pm_power_supplied_to),
	chip->wireless_psy.properties = pm_power_props_wireless,
	chip->wireless_psy.num_properties = ARRAY_SIZE(pm_power_props_wireless),
	chip->wireless_psy.get_property = pm_power_get_property_wireless,
#endif
#ifdef CONFIG_LGE_PM
	chip->dc_psy.name = "ac",
#else
	chip->dc_psy.name = "pm8921-dc",
#endif
	chip->dc_psy.type = POWER_SUPPLY_TYPE_MAINS,
	chip->dc_psy.supplied_to = pm_power_supplied_to,
	chip->dc_psy.num_supplicants = ARRAY_SIZE(pm_power_supplied_to),
	chip->dc_psy.properties = pm_power_props_mains,
	chip->dc_psy.num_properties = ARRAY_SIZE(pm_power_props_mains),
	chip->dc_psy.get_property = pm_power_get_property_mains,
/* BEGIN : janghyun.baek@lge.com 2012-05-06 for TA/USB recognition */
#ifdef CONFIG_LGE_PM
	chip->dc_psy.set_property = pm_power_set_property_mains,
#endif
/* END : janghyun.baek@lge.com 2012-05-06 */

	chip->batt_psy.name = "battery",
	chip->batt_psy.type = POWER_SUPPLY_TYPE_BATTERY,
	chip->batt_psy.properties = msm_batt_power_props,
	chip->batt_psy.num_properties = ARRAY_SIZE(msm_batt_power_props),
	chip->batt_psy.get_property = pm_batt_power_get_property,
#ifndef CONFIG_LGE_WIRELESS_CHARGER
	chip->batt_psy.external_power_changed = pm_batt_external_power_changed,
#else
	chip->batt_psy.external_power_changed = NULL,


	rc = power_supply_register(chip->dev, &chip->wireless_psy);
	if (rc < 0) {
		pr_err("power_supply_register wireless failed rc = %d\n", rc);
		goto free_chip;
	}
#endif

	rc = power_supply_register(chip->dev, &chip->usb_psy);
	if (rc < 0) {
		pr_err("power_supply_register usb failed rc = %d\n", rc);
#ifdef CONFIG_LGE_WIRELESS_CHARGER
		goto unregister_wireless;
#else
		goto free_chip;
#endif
	}

	rc = power_supply_register(chip->dev, &chip->dc_psy);
	if (rc < 0) {
		pr_err("power_supply_register usb failed rc = %d\n", rc);
		goto unregister_usb;
	}

	rc = power_supply_register(chip->dev, &chip->batt_psy);
	if (rc < 0) {
		pr_err("power_supply_register batt failed rc = %d\n", rc);
		goto unregister_dc;
	}

	platform_set_drvdata(pdev, chip);
	the_chip = chip;

	wake_lock_init(&chip->eoc_wake_lock, WAKE_LOCK_SUSPEND, "pm8921_eoc");
#if defined(CONFIG_LGE_PM) && !defined(CONFIG_BATTERY_MAX17043)
	wake_lock_init(&chip->batt_alarm_wake_up,
			WAKE_LOCK_SUSPEND, "pm8921_batt_alarm_wakeup");
#endif
#ifdef CONFIG_LGE_PM
	wake_lock_init(&uevent_wake_lock, WAKE_LOCK_SUSPEND, "pm8921_chg_uevent");
#endif

	INIT_DELAYED_WORK(&chip->eoc_work, eoc_worker);
	INIT_DELAYED_WORK(&chip->vin_collapse_check_work,
						vin_collapse_check_worker);

#ifdef CONFIG_MACH_LGE_TEMP
	INIT_WORK(&chip->unplug_ovp_fet_open_work,
                                       unplug_ovp_fet_open_worker);
#endif
	INIT_DELAYED_WORK(&chip->unplug_check_work, unplug_check_worker);

	rc = request_irqs(chip, pdev);
	if (rc) {
		pr_err("couldn't register interrupts rc=%d\n", rc);
		goto unregister_batt;
	}

	enable_irq_wake(chip->pmic_chg_irq[USBIN_VALID_IRQ]);
	enable_irq_wake(chip->pmic_chg_irq[USBIN_OV_IRQ]);
	enable_irq_wake(chip->pmic_chg_irq[USBIN_UV_IRQ]);
	enable_irq_wake(chip->pmic_chg_irq[BAT_TEMP_OK_IRQ]);
	enable_irq_wake(chip->pmic_chg_irq[VBATDET_LOW_IRQ]);
	enable_irq_wake(chip->pmic_chg_irq[FASTCHG_IRQ]);
#ifdef CONFIG_LGE_WIRELESS_CHARGER
	enable_irq_wake(chip->pmic_chg_irq[DCIN_VALID_IRQ]);
	enable_irq_wake(chip->pmic_chg_irq[DCIN_OV_IRQ]);
	enable_irq_wake(chip->pmic_chg_irq[DCIN_UV_IRQ]);
#endif
	/*
	 * if both the cool_temp_dc and warm_temp_dc are invalid device doesnt
	 * care for jeita compliance
	 */

/* BEGIN : janghyun.baek@lge.com 2012-03-10 LGE devices use temp scenario, don't use BTM. */
#ifdef CONFIG_MACH_LGE
	if (!(chip->cool_temp_dc == 0 && chip->warm_temp_dc == 0)) {
#else  /* QCT_ORG */
	if (!(chip->cool_temp_dc == INT_MIN && chip->warm_temp_dc == INT_MIN)) {
#endif
/* END : janghyun.baek@lge.com 2012-03-10 */

		rc = configure_btm(chip);
		if (rc) {
			pr_err("couldn't register with btm rc=%d\n", rc);
			goto free_irq;
		}
	}

	create_debugfs_entries(chip);

	INIT_WORK(&chip->bms_notify.work, bms_notify);
	INIT_WORK(&chip->battery_id_valid_work, battery_id_valid);

//#ifdef CONFIG_LGE_WIRELESS_CHARGER
	wireless_probe(chip);
//#endif

	/* determine what state the charger is in */
	determine_initial_state(chip);

#ifdef CONFIG_MACH_LGE
	rc = device_create_file(&pdev->dev, &dev_attr_at_current);
	rc = device_create_file(&pdev->dev, &dev_attr_at_charge);
	rc = device_create_file(&pdev->dev, &dev_attr_at_chcomp);
	rc = device_create_file(&pdev->dev, &dev_attr_at_pmrst);
	rc = device_create_file(&pdev->dev, &dev_attr_at_vcoin);
	rc = device_create_file(&pdev->dev, &dev_attr_at_battemp);
#endif
#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
	rc = device_create_file(&pdev->dev, &dev_attr_batt_temp_adc);
#endif
	if (chip->update_time) {
		INIT_DELAYED_WORK(&chip->update_heartbeat_work,
							update_heartbeat);
		schedule_delayed_work(&chip->update_heartbeat_work,
				      round_jiffies_relative(msecs_to_jiffies
							(chip->update_time)));
	}

#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
		wake_lock_init(&chip->monitor_batt_temp_wake_lock, WAKE_LOCK_SUSPEND,"pm8921_monitor_batt_temp");
		INIT_DELAYED_WORK(&chip->monitor_batt_temp_work,
							monitor_batt_temp);
		rc = schedule_delayed_work(&chip->monitor_batt_temp_work,
					  round_jiffies_relative(msecs_to_jiffies
							(60000)));
#endif
/* 120429 mansu.lee@lge.com Optimization GPIOs Set for Rock-Bottom */
#ifdef CONFIG_MACH_LGE
	gpio_tlmm_config(GPIO_CFG(chip->batt_id_gpio, 0,
		GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(chip->batt_id_pu_gpio, 0,
		GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
#endif
/* 120429 mansu.lee@lge.com */

/* Battery alarm for the model using BMS */
#if defined(CONFIG_LGE_PM) && !defined(CONFIG_BATTERY_MAX17043)
	rc = pm8xxx_batt_alarm_disable(1);	// PM8XXX_BATT_ALARM_UPPER_COMPARATOR - 1

	if (rc)
		pr_err(	"[batt_alarm] pm8xxx_batt_alarm_disable : unable to set alarm state. rc = %d\n", rc);

	rc = pm8xxx_batt_alarm_disable(1);	// PM8XXX_BATT_ALARM_LOWER_COMPARATOR - 0

	if (rc)
		pr_err(	"[batt_alarm] pm8xxx_batt_alarm_enable : unable to set alarm state. rc = %d\n", rc);

	rc = pm8xxx_batt_alarm_threshold_set(1, BATTERY_UPPER_THRESHOLD);	// PM8XXX_BATT_ALARM_UPPER_COMPARATOR - 1

	if (rc)
		pr_err("[batt_alarm] threshold_set failed 1, rc=%d\n", rc);

	rc = pm8xxx_batt_alarm_threshold_set(0, BATTERY_LOW_THRESHOLD);	// PM8XXX_BATT_ALARM_LOWER_COMPARATOR - 0

	if (rc)
		pr_err("[batt_alarm] threshold_set failed 0, rc=%d\n", rc);

	rc = pm8xxx_batt_alarm_hold_time_set(7);	// PM8XXX_BATT_ALARM_HOLD_TIME_16_MS - 7

	if (rc)
		pr_err("[batt_alarm] pm8xxx_batt_alarm_hold_time_set : unable to set batt alarm hold time. rc = %d\n", rc);

	/* PWM enabled at 2Hz */
	rc = pm8xxx_batt_alarm_pwm_rate_set(1, 7, 4);

	if (rc)
		pr_err("[batt_alarm] pm8xxx_batt_alarm_pwm_rate_set : unable to set batt alarm pwm rate. rc = %d\n", rc);

	rc = pm8xxx_batt_alarm_register_notifier(&alarm_notifier);

	if (rc)
		pr_err("[batt_alarm] pm8xxx_batt_alarm_register_notifier : unable to register alarm notifier. rc = %d\n", rc);
#endif


	return 0;

free_irq:
	free_irqs(chip);
unregister_batt:
	power_supply_unregister(&chip->batt_psy);
unregister_dc:
	power_supply_unregister(&chip->dc_psy);
unregister_usb:
	power_supply_unregister(&chip->usb_psy);
#ifdef CONFIG_LGE_WIRELESS_CHARGER
unregister_wireless:
	power_supply_unregister(&chip->wireless_psy);
#endif
free_chip:
	kfree(chip);
	return rc;
}

static int __devexit pm8921_charger_remove(struct platform_device *pdev)
{
	struct pm8921_chg_chip *chip = platform_get_drvdata(pdev);

#ifdef CONFIG_MACH_LGE
	device_remove_file(&pdev->dev, &dev_attr_at_current); /* sungsookim */
	device_remove_file(&pdev->dev, &dev_attr_at_charge);
	device_remove_file(&pdev->dev, &dev_attr_at_chcomp);
	device_remove_file(&pdev->dev, &dev_attr_at_pmrst);
	device_remove_file(&pdev->dev, &dev_attr_at_vcoin);
	device_remove_file(&pdev->dev, &dev_attr_at_battemp);
#endif
#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
    device_remove_file(&pdev->dev, &dev_attr_batt_temp_adc);
#endif
#ifdef CONFIG_LGE_PM
	wake_lock_destroy(&uevent_wake_lock);
#endif
	free_irqs(chip);
	platform_set_drvdata(pdev, NULL);
	the_chip = NULL;
	kfree(chip);
	return 0;
}
static const struct dev_pm_ops pm8921_pm_ops = {
	.suspend	= pm8921_charger_suspend,
	.suspend_noirq  = pm8921_charger_suspend_noirq,
	.resume_noirq   = pm8921_charger_resume_noirq,
	.resume		= pm8921_charger_resume,
};
static struct platform_driver pm8921_charger_driver = {
	.probe		= pm8921_charger_probe,
	.remove		= __devexit_p(pm8921_charger_remove),
	.driver		= {
			.name	= PM8921_CHARGER_DEV_NAME,
			.owner	= THIS_MODULE,
			.pm	= &pm8921_pm_ops,
	},
};

static int __init pm8921_charger_init(void)
{
	return platform_driver_register(&pm8921_charger_driver);
}

static void __exit pm8921_charger_exit(void)
{
	platform_driver_unregister(&pm8921_charger_driver);
}

late_initcall(pm8921_charger_init);
module_exit(pm8921_charger_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("PMIC8921 charger/battery driver");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:" PM8921_CHARGER_DEV_NAME);
