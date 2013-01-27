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

#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/bootmem.h>
#include <asm/mach-types.h>
#include <mach/msm_bus_board.h>
#include <mach/msm_memtypes.h>
#include <mach/board.h>
#include <mach/gpio.h>
#include <mach/gpiomux.h>
#include <linux/ion.h>
#include <mach/ion.h>

#include "devices.h"
#ifndef CONFIG_MACH_LGE

#endif

#include <linux/fb.h>
#include "../../../../drivers/video/msm/msm_fb.h"
#include "../../../../drivers/video/msm/msm_fb_def.h"
#include "../../../../drivers/video/msm/mipi_dsi.h"

#include <mach/board_lge.h>
#include CONFIG_BOARD_HEADER_FILE


#ifdef CONFIG_LGE_KCAL
#ifdef CONFIG_LGE_QC_LCDC_LUT
extern int set_qlut_kcal_values(int kcal_r, int kcal_g, int kcal_b);
extern int refresh_qlut_display(void);
#else
#error only kcal by Qucalcomm LUT is supported now!!!
#endif
#endif

#ifdef CONFIG_FB_MSM_TRIPLE_BUFFER
#define MSM_FB_PRIM_BUF_SIZE (LCD_RESOLUTION_X * LCD_RESOLUTION_Y * 4 * 3)
/*  4(bpp) x 3(pages) */
#else
#define MSM_FB_PRIM_BUF_SIZE (LCD_RESOLUTION_X * LCD_RESOLUTION_Y * 4 * 2)
/*  4(bpp) x 2(pages) */
#endif

#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
#define MSM_FB_EXT_BUF_SIZE \
		(roundup((1920 * 1088 * 2), 4096) * 1) /* 2 bpp x 1 page */
#elif defined(CONFIG_FB_MSM_TVOUT)
#define MSM_FB_EXT_BUF_SIZE \
		(roundup((720 * 576 * 2), 4096) * 2) /* 2 bpp x 2 pages */
#else
#define MSM_FB_EXT_BUF_SIZE	0
#endif

/* Note: must be multiple of 4096 */
#define MSM_FB_SIZE roundup(MSM_FB_PRIM_BUF_SIZE + MSM_FB_EXT_BUF_SIZE, 4096)

#ifdef CONFIG_FB_MSM_OVERLAY0_WRITEBACK
#define MSM_FB_OVERLAY0_WRITEBACK_SIZE roundup((LCD_RESOLUTION_X * LCD_RESOLUTION_Y * 3 * 2), 4096)
#else
#define MSM_FB_OVERLAY0_WRITEBACK_SIZE (0)
#endif  /* CONFIG_FB_MSM_OVERLAY0_WRITEBACK */

#ifdef CONFIG_FB_MSM_OVERLAY1_WRITEBACK
#define MSM_FB_OVERLAY1_WRITEBACK_SIZE roundup((1920 * 1088 * 3 * 2), 4096)
#else
#define MSM_FB_OVERLAY1_WRITEBACK_SIZE (0)
#endif  /* CONFIG_FB_MSM_OVERLAY1_WRITEBACK */

#define MDP_VSYNC_GPIO 0

#define MIPI_CMD_NOVATEK_QHD_PANEL_NAME	"mipi_cmd_novatek_qhd"
#define MIPI_VIDEO_NOVATEK_QHD_PANEL_NAME	"mipi_video_novatek_qhd"
#define MIPI_VIDEO_TOSHIBA_WSVGA_PANEL_NAME	"mipi_video_toshiba_wsvga"
#define MIPI_VIDEO_TOSHIBA_WUXGA_PANEL_NAME	"mipi_video_toshiba_wuxga"
#define MIPI_VIDEO_CHIMEI_WXGA_PANEL_NAME	"mipi_video_chimei_wxga"
#define MIPI_VIDEO_SIMULATOR_VGA_PANEL_NAME	"mipi_video_simulator_vga"
#define MIPI_CMD_RENESAS_FWVGA_PANEL_NAME	"mipi_cmd_renesas_fwvga"
#define HDMI_PANEL_NAME	"hdmi_msm"
#define TVOUT_PANEL_NAME	"tvout_msm"

#ifdef CONFIG_FB_MSM_HDMI_AS_PRIMARY
unsigned char hdmi_is_primary = 1;
#else
unsigned char hdmi_is_primary;
#endif
#define TUNING_BUFSIZE 4096
#define TUNING_REGSIZE 40
#define TUNING_REGNUM 10
#define LCD_GAMMA 0
#if defined(CONFIG_LGE_BACKLIGHT_CABC)
#define CABC_POWERON_OFFSET 4 /* offset from lcd display on cmds */

#define CABC_OFF 0
#define CABC_ON 1

#define CABC_10 1
#define CABC_20 2
#define CABC_30 3
#define CABC_40 4
#define CABC_50 5

#define CABC_DEFAULT CABC_10

#if defined (CONFIG_LGE_BACKLIGHT_CABC_DEBUG)
static int lgit_cabc_index = CABC_DEFAULT;
#endif /* CONFIG_LGE_BACKLIGHT_CABC_DEBUG */

#endif /* CONFIG_LGE_BACKLIGHT_CABC */

static struct resource msm_fb_resources[] = {
	{
		.flags = IORESOURCE_DMA,
	}
};

#ifndef CONFIG_MACH_LGE
#ifndef CONFIG_FB_MSM_MIPI_PANEL_DETECT
static void set_mdp_clocks_for_wuxga(void);
#endif
#endif

static int msm_fb_detect_panel(const char *name)
{
	return 0;
}


static struct msm_fb_platform_data msm_fb_pdata = {
	.detect_client = msm_fb_detect_panel,
};

static struct platform_device msm_fb_device = {
	.name   = "msm_fb",
	.id     = 0,
	.num_resources     = ARRAY_SIZE(msm_fb_resources),
	.resource          = msm_fb_resources,
	.dev.platform_data = &msm_fb_pdata,
};

#ifndef CONFIG_MACH_LGE
static void mipi_dsi_panel_pwm_cfg(void)
{
	int rc;
	static int mipi_dsi_panel_gpio_configured;
	static struct pm_gpio pwm_enable = {
		.direction        = PM_GPIO_DIR_OUT,
		.output_buffer    = PM_GPIO_OUT_BUF_CMOS,
		.output_value     = 1,
		.pull             = PM_GPIO_PULL_NO,
		.vin_sel          = PM_GPIO_VIN_VPH,
		.out_strength     = PM_GPIO_STRENGTH_HIGH,
		.function         = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol      = 0,
		.disable_pin      = 0,
	};
	static struct pm_gpio pwm_mode = {
		.direction        = PM_GPIO_DIR_OUT,
		.output_buffer    = PM_GPIO_OUT_BUF_CMOS,
		.output_value     = 0,
		.pull             = PM_GPIO_PULL_NO,
		.vin_sel          = PM_GPIO_VIN_S4,
		.out_strength     = PM_GPIO_STRENGTH_HIGH,
		.function         = PM_GPIO_FUNC_2,
		.inv_int_pol      = 0,
		.disable_pin      = 0,
	};

	if (mipi_dsi_panel_gpio_configured == 0) {
		/* pm8xxx: gpio-21, Backlight Enable */
		rc = pm8xxx_gpio_config(PM8921_GPIO_PM_TO_SYS(21),
					&pwm_enable);
		if (rc != 0)
			pr_err("%s: pwm_enabled failed\n", __func__);

		/* pm8xxx: gpio-24, Bl: Off, PWM mode */
		rc = pm8xxx_gpio_config(PM8921_GPIO_PM_TO_SYS(24),
					&pwm_mode);
		if (rc != 0)
			pr_err("%s: pwm_mode failed\n", __func__);

		mipi_dsi_panel_gpio_configured++;
	}
}
#endif

static bool dsi_power_on;

/* LGE_CHANGE
  * LG Display 4.0' WVGA for l_dcm
  * kyunghoo.ryu@lge.com
  */
static int mipi_dsi_panel_power(int on)
{
	static struct regulator *reg_l8, *reg_l2, *reg_lvs6;
	static int gpio43 = PM8921_GPIO_PM_TO_SYS(43);
	int rc;


	pr_debug("%s: state : %d\n", __func__, on);

	if (!dsi_power_on) {

		reg_l8 = regulator_get(&msm_mipi_dsi1_device.dev,
				"dsi_vdc");
		if (IS_ERR(reg_l8)) {
			pr_err("could not get 8921_l8, rc = %ld\n",
				PTR_ERR(reg_l8));
			return -ENODEV;
		}
		reg_lvs6 = regulator_get(&msm_mipi_dsi1_device.dev,
				"8921_lvs6");
		if (IS_ERR(reg_lvs6)) {
			pr_err("could not get 8921_lvs6, rc = %ld\n",
				PTR_ERR(reg_lvs6));
			return -ENODEV;
		}
		reg_l2 = regulator_get(&msm_mipi_dsi1_device.dev,
				"dsi_vdda");
		if (IS_ERR(reg_l2)) {
			pr_err("could not get 8921_l2, rc = %ld\n",
				PTR_ERR(reg_l2));
			return -ENODEV;
		}
		rc = regulator_set_voltage(reg_l8, 2800000, 2800000);
		if (rc) {
			pr_err("set_voltage l8 failed, rc=%d\n", rc);
			return -EINVAL;
		}
		rc = regulator_set_voltage(reg_l2, 1200000, 1200000);
		if (rc) {
			pr_err("set_voltage l2 failed, rc=%d\n", rc);
			return -EINVAL;
		}

		/* VREG_2P8_LCD_VCI enable - kyunghoo.ryu@lge.com */
		rc = gpio_request(LCD_VCI_EN_GPIO, "LCD_VCI_EN_GPIO");
		if (rc) {
			pr_err("'%s'(%d) gpio_request failed, rc=%d\n",
				"LCD_VCI_EN_GPIO", LCD_VCI_EN_GPIO, rc);		
		}
		
		gpio_tlmm_config(GPIO_CFG(LCD_VCI_EN_GPIO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	GPIO_CFG_ENABLE);
		
		rc = gpio_request(gpio43, "disp_rst_n");
		if (rc) {
			pr_err("request gpio 43 failed, rc=%d\n", rc);
			return -ENODEV;
		}
		dsi_power_on = true;
	}

	if (on) {
		rc = regulator_set_optimum_mode(reg_l8, 100000);
		if (rc < 0) {
			pr_err("set_optimum_mode l8 failed, rc=%d\n", rc);
			return -EINVAL;
		}

		rc = regulator_set_optimum_mode(reg_l2, 100000);
		if (rc < 0) {
			pr_err("set_optimum_mode l2 failed, rc=%d\n", rc);
			return -EINVAL;
		}

		rc = regulator_enable(reg_l8);
		if (rc) {
			pr_err("enable l8 failed, rc=%d\n", rc);
			return -ENODEV;
		}

		rc = regulator_enable(reg_lvs6);
		if (rc) {
			pr_err("enable lvs6 failed, rc=%d\n", rc);
			return -ENODEV;
		}

		rc = gpio_direction_output(LCD_VCI_EN_GPIO, 1);
		mdelay(1);

		rc = regulator_enable(reg_l2);
		if (rc) {
			pr_err("enable l2 failed, rc=%d\n", rc);
			return -ENODEV;
		}
	} else {

		rc = regulator_disable(reg_l8);
		if (rc) {
			pr_err("disable reg_l8 failed, rc=%d\n", rc);
			return -ENODEV;
		}
		rc = regulator_disable(reg_lvs6);
		if (rc) {
			pr_err("disable reg_lvs6 failed, rc=%d\n", rc);
			return -ENODEV;
		}
		rc = regulator_disable(reg_l2);
		if (rc) {
			pr_err("enable l2 failed, rc=%d\n", rc);
			return -ENODEV;
		}

		/* LCD Reset LOW */
		gpio_direction_output(gpio43, 0);

		/* LCD VCI EN LOW */
		rc = gpio_direction_output(LCD_VCI_EN_GPIO, 0);
		
		rc = regulator_set_optimum_mode(reg_l8, 100);
		if (rc < 0) {
			pr_err("set_optimum_mode l8 failed, rc=%d\n", rc);
			return -EINVAL;
		}
		rc = regulator_set_optimum_mode(reg_l2, 100);
		if (rc < 0) {
			pr_err("set_optimum_mode l2 failed, rc=%d\n", rc);
			return -EINVAL;
		}		
	}
	return 0;
}

static struct mipi_dsi_platform_data mipi_dsi_pdata = {
	.vsync_gpio = MDP_VSYNC_GPIO,
	.dsi_power_save = mipi_dsi_panel_power,
};

#ifdef CONFIG_MSM_BUS_SCALING

static struct msm_bus_vectors rotator_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_ROTATOR,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};

static struct msm_bus_vectors rotator_ui_vectors[] = {
	{
		.src = MSM_BUS_MASTER_ROTATOR,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = (1024 * 600 * 4 * 2 * 60),
		.ib  = (1024 * 600 * 4 * 2 * 60 * 1.5),
	},
};

static struct msm_bus_vectors rotator_vga_vectors[] = {
	{
		.src = MSM_BUS_MASTER_ROTATOR,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = (640 * 480 * 2 * 2 * 30),
		.ib  = (640 * 480 * 2 * 2 * 30 * 1.5),
	},
};
static struct msm_bus_vectors rotator_720p_vectors[] = {
	{
		.src = MSM_BUS_MASTER_ROTATOR,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = (1280 * 736 * 2 * 2 * 30),
		.ib  = (1280 * 736 * 2 * 2 * 30 * 1.5),
	},
};

static struct msm_bus_vectors rotator_1080p_vectors[] = {
	{
		.src = MSM_BUS_MASTER_ROTATOR,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = (1920 * 1088 * 2 * 2 * 30),
		.ib  = (1920 * 1088 * 2 * 2 * 30 * 1.5),
	},
};

static struct msm_bus_paths rotator_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(rotator_init_vectors),
		rotator_init_vectors,
	},
	{
		ARRAY_SIZE(rotator_ui_vectors),
		rotator_ui_vectors,
	},
	{
		ARRAY_SIZE(rotator_vga_vectors),
		rotator_vga_vectors,
	},
	{
		ARRAY_SIZE(rotator_720p_vectors),
		rotator_720p_vectors,
	},
	{
		ARRAY_SIZE(rotator_1080p_vectors),
		rotator_1080p_vectors,
	},
};

struct msm_bus_scale_pdata rotator_bus_scale_pdata = {
	rotator_bus_scale_usecases,
	ARRAY_SIZE(rotator_bus_scale_usecases),
	.name = "rotator",
};

static struct msm_bus_vectors mdp_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};

#ifdef CONFIG_FB_MSM_HDMI_AS_PRIMARY
static struct msm_bus_vectors hdmi_as_primary_vectors[] = {
	/* If HDMI is used as primary */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 2000000000,
		.ib = 2000000000,
	},
};
static struct msm_bus_paths mdp_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(mdp_init_vectors),
		mdp_init_vectors,
	},
	{
		ARRAY_SIZE(hdmi_as_primary_vectors),
		hdmi_as_primary_vectors,
	},
	{
		ARRAY_SIZE(hdmi_as_primary_vectors),
		hdmi_as_primary_vectors,
	},
	{
		ARRAY_SIZE(hdmi_as_primary_vectors),
		hdmi_as_primary_vectors,
	},
	{
		ARRAY_SIZE(hdmi_as_primary_vectors),
		hdmi_as_primary_vectors,
	},
	{
		ARRAY_SIZE(hdmi_as_primary_vectors),
		hdmi_as_primary_vectors,
	},
};
#else
static struct msm_bus_vectors mdp_ui_vectors[] = {
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 216000000 * 2,
		.ib = 270000000 * 2,
	},
};

static struct msm_bus_vectors mdp_vga_vectors[] = {
	/* VGA and less video */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 216000000 * 2,
		.ib = 270000000 * 2,
	},
};

static struct msm_bus_vectors mdp_720p_vectors[] = {
	/* 720p and less video */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 230400000 * 2,
		.ib = 288000000 * 2,
	},
};

static struct msm_bus_vectors mdp_1080p_vectors[] = {
	/* 1080p and less video */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 334080000 * 2,
		.ib = 417600000 * 2,
	},
};

static struct msm_bus_paths mdp_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(mdp_init_vectors),
		mdp_init_vectors,
	},
	{
		ARRAY_SIZE(mdp_ui_vectors),
		mdp_ui_vectors,
	},
	{
		ARRAY_SIZE(mdp_ui_vectors),
		mdp_ui_vectors,
	},
	{
		ARRAY_SIZE(mdp_vga_vectors),
		mdp_vga_vectors,
	},
	{
		ARRAY_SIZE(mdp_720p_vectors),
		mdp_720p_vectors,
	},
	{
		ARRAY_SIZE(mdp_1080p_vectors),
		mdp_1080p_vectors,
	},
};
#endif

static struct msm_bus_scale_pdata mdp_bus_scale_pdata = {
	mdp_bus_scale_usecases,
	ARRAY_SIZE(mdp_bus_scale_usecases),
	.name = "mdp",
};

#endif

static int mdp_core_clk_rate_table[] = {
	128000000,/*85330000,*/
	128000000,/*85330000,*/
	160000000,
	200000000,
};

struct msm_fb_info_st {
	unsigned int width_mm;
	unsigned int height_mm;
};

static struct msm_fb_info_st msm_fb_info_data = {
	.width_mm = MSM_FB_WIDTH_MM,
	.height_mm = MSM_FB_HEIGHT_MM
};

static int msm_fb_event_notify(struct notifier_block *self,
		unsigned long action, void *data)
{
	struct fb_event *event = data;
	struct fb_info *info = event->info;
	struct msm_fb_info_st *fb_info_mm = &msm_fb_info_data;
	int ret = 0;

	switch (action) {
	case FB_EVENT_FB_REGISTERED:
		info->var.width = fb_info_mm->width_mm;
		info->var.height = fb_info_mm->height_mm;
		break;
	}
	return ret;
}

static struct notifier_block msm_fb_event_notifier = {
	.notifier_call = msm_fb_event_notify,
};

static struct msm_panel_common_pdata mdp_pdata = {
	.gpio = MDP_VSYNC_GPIO,
	.mdp_core_clk_rate = 128000000, /*85330000,*/
	.mdp_core_clk_table = mdp_core_clk_rate_table,
	.num_mdp_clk = ARRAY_SIZE(mdp_core_clk_rate_table),
#ifdef CONFIG_MSM_BUS_SCALING
	.mdp_bus_scale_table = &mdp_bus_scale_pdata,
#endif
	.mdp_rev = MDP_REV_42,
#ifdef CONFIG_MSM_MULTIMEDIA_USE_ION
	.mem_hid = BIT(ION_CP_MM_HEAP_ID),
#else
	.mem_hid = MEMTYPE_EBI1,
#endif
	.cont_splash_enabled = 0x00,
};

#ifndef CONFIG_MACH_LGE
#ifndef CONFIG_FB_MSM_MIPI_PANEL_DETECT
/**
 * Set MDP clocks to high frequency to avoid DSI underflow
 * when using high resolution 1200x1920 WUXGA panels
 */
static void set_mdp_clocks_for_wuxga(void)
{
	int i;

	mdp_ui_vectors[0].ab = 2000000000;
	mdp_ui_vectors[0].ib = 2000000000;
	mdp_vga_vectors[0].ab = 2000000000;
	mdp_vga_vectors[0].ib = 2000000000;
	mdp_720p_vectors[0].ab = 2000000000;
	mdp_720p_vectors[0].ib = 2000000000;
	mdp_1080p_vectors[0].ab = 2000000000;
	mdp_1080p_vectors[0].ib = 2000000000;

	mdp_pdata.mdp_core_clk_rate = 200000000;

	for (i = 0; i < ARRAY_SIZE(mdp_core_clk_rate_table); i++)
		mdp_core_clk_rate_table[i] = 200000000;

}
#endif
#endif

void __init msm8960_mdp_writeback(struct memtype_reserve* reserve_table)
{
	mdp_pdata.ov0_wb_size = MSM_FB_OVERLAY0_WRITEBACK_SIZE;
	mdp_pdata.ov1_wb_size = MSM_FB_OVERLAY1_WRITEBACK_SIZE;
#if defined(CONFIG_ANDROID_PMEM) && !defined(CONFIG_MSM_MULTIMEDIA_USE_ION)
	reserve_table[mdp_pdata.mem_hid].size +=
		mdp_pdata.ov0_wb_size;
	reserve_table[mdp_pdata.mem_hid].size +=
		mdp_pdata.ov1_wb_size;
#endif
}

/*  LGE_CHANGE
 *
 *	LM3533TMX BL driver for l_dcm
 *	2011-11-23 kyunghoo.ryu@lge.com
*/

#ifdef CONFIG_LGE_BACKLIGHT_LM3533
extern void lm3533_lcd_backlight_set_level(int level);

#ifdef CONFIG_FB_MSM_MIPI_DSI_LGIT
static int mipi_lgit_backlight_level(int level, int max, int min)
{
	lm3533_lcd_backlight_set_level(level);
	return 0;
}
#if defined(CONFIG_FB_MSM_MIPI_LGIT_CMD_WVGA_INVERSE_PT_PANEL) ||\
	defined(CONFIG_FB_MSM_MIPI_LGIT_VIDEO_WVGA_INVERSE_PT_PANEL)

/* LG Display 4.0" WVGA for l_dcm (CMD Mode)
  * Rotate Display output by l_dcm h/w implementation
  * 2011-11-24 Kyunghoo.ryu@lge.com
  */
#if defined(CONFIG_FB_MSM_MIPI_LGIT_VIDEO_WVGA_INVERSE_PT_PANEL)
static char video_switch[] = {0x01, 0x47};
#endif

/* LG-4572B only for Rev.A and Rev.B */
static char hrx_to_old					[ 2] = {0x03, 0x00};
static char inversion_off_old			[ 2] = {0x20, 0x00};
static char tear_on_old					[ 2] = {0x35, 0x00};
static char set_address_mode_old		[ 2] = {0x36, 0x02};			/* Flip Horizontal Only (cause Tearing problem) - Kyunghoo.ryu@lge.com */
static char if_pixel_format_old			[ 2] = {0x3A, 0x77};
#if defined(CONFIG_FB_MSM_MIPI_LGIT_VIDEO_WVGA_INVERSE_PT_PANEL)
static char rgb_interface_setting_old	[  ] = {0xB1, 0x06, 0x43, 0x0A};
#endif
static char page_address_set_old		[ 5] = {0x2B, 0x00, 0x00, 0x03, 0x1F};
static char panel_char_setting_old		[ 3] = {0xB2, 0x00, 0xC8};
static char panel_drive_setting_old		[ 2] = {0xB3, 0x00};
static char display_mode_ctrl_old		[ 2] = {0xB4, 0x04};
static char display_ctrl1_old			[ 6] = {0xB5, 0x42, 0x10, 0x10, 0x00, 0x20};
static char display_ctrl2_old			[ 7] = {0xB6, 0x0B, 0x0F, 0x02, 0x40, 0x10, 0xE8};
#if defined(CONFIG_FB_MSM_MIPI_LGIT_CMD_WVGA_INVERSE_PT_PANEL)
static char display_ctrl3_old			[ 6] = {0xB7, 0x48, 0x06, 0x2E, 0x00, 0x00};
#endif

static char osc_setting_old				[ 3] = {0xC0, 0x01, 0x15};

static char power_ctrl3_old				[ 6] = {0xC3, 0x07, 0x03, 0x04, 0x04, 0x04};
static char power_ctrl4_old				[ 7] = {0xC4, 0x12, 0x24, 0x18, 0x18, 0x05, 0x49};
static char power_ctrl5_old				[ 2] = {0xC5, 0x69};
static char power_ctrl6_old				[ 3] = {0xC6, 0x41, 0x63};

static char exit_sleep_old				[ 2] = {0x11, 0x00};
static char display_on_old				[ 2] = {0x29, 0x00};
static char enter_sleep_old				[ 2] = {0x10, 0x00};
static char display_off_old				[ 2] = {0x28, 0x00};
static char deep_standby_old			[ 2] = {0xC1, 0x01};

/* LGE_CHANGE_S LG-4573B H/W Rev.C or upper revision, jamin.koo@lge.com, 2011.02.27 */
static char hrx_to						[ 2] = {0x03, 0x00};
static char inversion_off				[ 1] = {0x20};
static char set_address_mode			[ 2] = {0x36, 0x02};         /* Flip Horizontal Only (cause Tearing problem) - Kyunghoo.ryu@lge.com */
static char if_pixel_format				[ 2] = {0x3A, 0x70};
/* LGE_CHANGE_S, Add CABC Code, jamin.koo@lge.com, 2012.03.30 */

#ifdef CONFIG_LGE_BACKLIGHT_CABC
static char cabc_51				[ 2] = {0x51,0xE6};	 /* LCD CABC CODE, Write Display Brightness */
static char cabc_53				[ 2] = {0x53,0x24};	 /* LCD CABC CODE, Write Control Display */
static char cabc_55				[ 2] = {0x55,0x01};	 /* LCD CABC CODE, Write Content Adaptive Brightness Control */
static char cabc_5e				[ 2] = {0x5E,0x33};	 /* LCD CABC CODE, Write CABC Minimum Brightness */

#ifdef CONFIG_LGE_BACKLIGHT_CABC_DEBUG
/* Write Display Brightness */
static char config_cabc_51[6][2] = {
	{0x51, 0x00},	/* off */
	{0x51, 0xE6},	/* 10%, 230 */
	{0x51, 0xCC},	/* 20%, 204 */
	{0x51, 0xB3},	/* 30%, 179 */
	{0x51, 0x99},	/* 40%, 153 */
	{0x51, 0x80}	/* 50%, 128 */
};

/* Write Control Display */
static char config_cabc_53[2][2] = {
	{0x53, 0x00},	/* off */
	{0x53, 0x24}	/* on */
};

/* Write Content Adaptive Brightness Control */
static char config_cabc_55[2][2] = {
	{0x55, 0x00},	/* off */
	{0x55, 0x01}	/* on */
};

/* Write CABC Minimum Brightness */
static char config_cabc_5e[6][2] = {
	{0x5E, 0x00},	/* off */
	{0x5E, 0x33},	/* 10% */
	{0x5E, 0x33},	/* 20% */
	{0x5E, 0x33},	/* 30% */
	{0x5E, 0x33},	/* 40% */
	{0x5E, 0x33}	/* 50% */
};
#endif /* CONFIG_LGE_BACKLIGHT_CABC_DEBUG */
#endif /* CONFIG_LGE_BACKLIGHT_CABC */
/* LGE_CHANGE_E, Add CABC Code, jamin.koo@lge.com, 2012.03.30 */

static char rgb_interface_setting		[ 4] = {0xB1, 0x06, 0x43, 0x0A};
static char panel_char_setting			[ 3] = {0xB2, 0x00, 0xC8};
static char panel_drive_setting			[ 2] = {0xB3, 0x00};
static char display_mode_ctrl			[ 2] = {0xB4, 0x04};
static char display_ctrl1				[ 6] = {0xB5, 0x40, 0x10, 0x10, 0x00, 0x00};
static char display_ctrl2				[ 7] = {0xB6, 0x0B, 0x0F, 0x02, 0x40, 0x10, 0xE8};

static char osc_setting					[ 3] = {0xC0, 0x01, 0x18};

static char power_ctrl3					[ 6] = {0xC3, 0x07, 0x0A, 0x0A, 0x0A, 0x02};
static char power_ctrl4					[ 7] = {0xC4, 0x12, 0x24, 0x18, 0x18, 0x05, 0x49};
static char power_ctrl5					[ 2] = {0xC5, 0x69};
static char power_ctrl6					[ 4] = {0xC6, 0x41, 0x63, 0x03};

static char p_gamma_r_setting			[10] = {0xD0, 0x00, 0x01, 0x64, 0x25, 0x07, 0x02, 0x61, 0x13, 0x03};
static char n_gamma_r_setting			[10] = {0xD1, 0x00, 0x01, 0x64, 0x25, 0x07, 0x02, 0x61, 0x13, 0x03};
static char p_gamma_g_setting			[10] = {0xD2, 0x00, 0x01, 0x64, 0x25, 0x07, 0x02, 0x61, 0x13, 0x03};
static char n_gamma_g_setting			[10] = {0xD3, 0x00, 0x01, 0x64, 0x25, 0x07, 0x02, 0x61, 0x13, 0x03};
static char p_gamma_b_setting			[10] = {0xD4, 0x00, 0x01, 0x64, 0x25, 0x07, 0x02, 0x61, 0x13, 0x03};
static char n_gamma_b_setting			[10] = {0xD5, 0x00, 0x01, 0x64, 0x25, 0x07, 0x02, 0x61, 0x13, 0x03};

static char exit_sleep					[ 1] = {0x11};
static char display_on					[ 1] = {0x29};
static char enter_sleep					[ 1] = {0x10};
static char display_off					[ 1] = {0x28};
/* LGE_CHANGE_E LG-4573B H/W Rev.C or upper revision, jamin.koo@lge.com, 2011.02.27 */


/* LG-4572B only for Rev.A and Rev.B */
/* initialize device */
static struct dsi_cmd_desc lgit_power_on_set_old[] = {
#if	defined(CONFIG_FB_MSM_MIPI_LGIT_VIDEO_WVGA_INVERSE_PT_PANEL)
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(video_switch), video_switch},
#endif
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(hrx_to_old), hrx_to_old},
	{DTYPE_DCS_WRITE,  1, 0, 0, 0, sizeof(inversion_off_old), inversion_off_old},
	{DTYPE_DCS_WRITE,  1, 0, 0, 0, sizeof(tear_on_old), tear_on_old},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(set_address_mode_old), set_address_mode_old},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(if_pixel_format_old),	if_pixel_format_old},
#if defined(CONFIG_FB_MSM_MIPI_LGIT_VIDEO_WVGA_INVERSE_PT_PANEL)
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(rgb_interface_setting_old), rgb_interface_setting_old},
#endif
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(page_address_set_old), page_address_set_old},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(panel_char_setting_old), panel_char_setting_old},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(panel_drive_setting_old), panel_drive_setting_old},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(display_mode_ctrl_old), display_mode_ctrl_old},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(display_ctrl1_old), display_ctrl1_old},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(display_ctrl2_old), display_ctrl2_old},
#if defined(CONFIG_FB_MSM_MIPI_LGIT_CMD_WVGA_INVERSE_PT_PANEL)
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(display_ctrl3_old), display_ctrl3_old},
#endif
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(osc_setting_old), osc_setting_old},

	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(power_ctrl3_old), power_ctrl3_old},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(power_ctrl4_old), power_ctrl4_old},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(power_ctrl5_old), power_ctrl5_old},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(power_ctrl6_old), power_ctrl6_old},

	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(p_gamma_r_setting), p_gamma_r_setting},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(n_gamma_r_setting), n_gamma_r_setting},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(p_gamma_g_setting), p_gamma_g_setting},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(n_gamma_g_setting), n_gamma_g_setting},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(p_gamma_b_setting), p_gamma_b_setting},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(n_gamma_b_setting), n_gamma_b_setting},

	{DTYPE_DCS_WRITE,  1, 0, 0, 100, sizeof(exit_sleep_old), exit_sleep_old},
	{DTYPE_DCS_WRITE,  1, 0, 0, 100, sizeof(display_on_old), display_on_old},
};

static struct dsi_cmd_desc lgit_power_off_set_old[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 0, sizeof(display_off_old), display_off_old},
	{DTYPE_DCS_WRITE, 1, 0, 0, 60, sizeof(enter_sleep_old), enter_sleep_old},
	{DTYPE_DCS_WRITE, 1, 0, 0, 0,   sizeof(deep_standby_old), deep_standby_old},
};

/* LGE_CHANGE_S LG-4573B H/W Rev.C or upper revision, jamin.koo@lge.com, 2011.02.27 */
/* initialize device */
static struct dsi_cmd_desc lgit_power_on_set[] = {

   {DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(hrx_to), hrx_to},
   {DTYPE_DCS_WRITE,  1, 0, 0, 0, sizeof(inversion_off), inversion_off},
   {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(set_address_mode), set_address_mode},
   {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(if_pixel_format),   if_pixel_format},

	/* LGE_CHANGE_S, Add CABC Code, jamin.koo@lge.com, 2012.03.30 */
#if defined(CONFIG_LGE_BACKLIGHT_CABC)
   {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(cabc_51), cabc_51},
   {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(cabc_53), cabc_53},
   {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(cabc_55), cabc_55},
   {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(cabc_5e), cabc_5e},
#endif /* CONFIG_LGE_BACKLIGHT_CABC */
	/* LGE_CHANGE_E, Add CABC Code, jamin.koo@lge.com, 2012.03.30 */

   {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(rgb_interface_setting), rgb_interface_setting},
   {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(panel_char_setting), panel_char_setting},
   {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(panel_drive_setting), panel_drive_setting},
   {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(display_mode_ctrl), display_mode_ctrl},
   {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(display_ctrl1), display_ctrl1},
   {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(display_ctrl2), display_ctrl2},

   {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(osc_setting), osc_setting},

   {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(power_ctrl3), power_ctrl3},
   {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(power_ctrl4), power_ctrl4},
   {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(power_ctrl5), power_ctrl5},
   {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(power_ctrl6), power_ctrl6},

   {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(p_gamma_r_setting), p_gamma_r_setting},
   {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(n_gamma_r_setting), n_gamma_r_setting},
   {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(p_gamma_g_setting), p_gamma_g_setting},
   {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(n_gamma_g_setting), n_gamma_g_setting},
   {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(p_gamma_b_setting), p_gamma_b_setting},
   {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(n_gamma_b_setting), n_gamma_b_setting},

   {DTYPE_DCS_WRITE,  1, 0, 0, 120, sizeof(exit_sleep), exit_sleep},
   {DTYPE_DCS_WRITE,  1, 0, 0, 40, sizeof(display_on), display_on},

};

/* LGE_CHANGE_E LG-4573B H/W Rev.C or upper revision, jamin.koo@lge.com, 2011.02.27 */
static struct dsi_cmd_desc lgit_power_off_set[] = {
   {DTYPE_DCS_WRITE, 1, 0, 0, 40, sizeof(display_off), display_off},
   {DTYPE_DCS_WRITE, 1, 0, 0, 10, sizeof(enter_sleep), enter_sleep},
};


#if defined(CONFIG_LGE_BACKLIGHT_CABC) && \
	defined(CONFIG_LGE_BACKLIGHT_CABC_DEBUG)

void set_lgit_cabc(int cabc_index)
{
	pr_info("%s! cabc_index: %d\n", __func__, cabc_index);
	switch(cabc_index) {
	case 0: /* CABC OFF */
		lgit_power_on_set[CABC_POWERON_OFFSET+2].payload =	config_cabc_55[CABC_OFF];
		break;
	case 1: /* 10% */
	case 2: /* 20% */
	case 3: /* 30% */
	case 4: /* 40% */
	case 5: /* 50% */
		{ /* CABC ON */
			lgit_power_on_set[CABC_POWERON_OFFSET].payload = config_cabc_51[cabc_index];
			lgit_power_on_set[CABC_POWERON_OFFSET+1].payload = config_cabc_53[CABC_ON];
			lgit_power_on_set[CABC_POWERON_OFFSET+2].payload = config_cabc_55[CABC_ON];
			lgit_power_on_set[CABC_POWERON_OFFSET+3].payload = config_cabc_5e[cabc_index];
		}
		break;
	default:
		printk("out of range cabc_index %d", cabc_index);
		return;
	}
	lgit_cabc_index = cabc_index;
	return;
}
EXPORT_SYMBOL(set_lgit_cabc);

int get_lgit_cabc(void)
{
	return lgit_cabc_index;
}
EXPORT_SYMBOL(get_lgit_cabc);

#endif /* CONFIG_LGE_BACKLIGHT_CABC && CONFIG_LGE_BACKLIGHT_CABC_DEBUG */

/* LG-4572B only for Rev.A and Rev.B */
static struct msm_panel_common_pdata mipi_lgit_pdata_old = {
	.backlight_level = mipi_lgit_backlight_level,
#if defined(CONFIG_FB_MSM_MIPI_LGIT_CMD_WVGA_INVERSE_PT_PANEL) ||\
			defined(CONFIG_FB_MSM_MIPI_LGIT_VIDEO_WVGA_INVERSE_PT_PANEL)
	.power_on_set = lgit_power_on_set_old,
	.power_off_set = lgit_power_off_set_old,
	.power_on_set_size = ARRAY_SIZE(lgit_power_on_set_old),
	.power_off_set_size = ARRAY_SIZE(lgit_power_off_set_old),
	.max_backlight_level = 0xFF,
#endif

#if defined (CONFIG_LGE_BACKLIGHT_LM3530)
	.max_backlight_level = 0x71,
#elif defined (CONFIG_LGE_BACKLIGHT_LM3533)
	.max_backlight_level = 0xFF,
#endif
};

/* LGE_CHANGE_S LG-4573B H/W Rev.C or upper revision, jamin.koo@lge.com, 2011.02.27 */
static struct msm_panel_common_pdata mipi_lgit_pdata = {
	.backlight_level = mipi_lgit_backlight_level,
#if defined(CONFIG_FB_MSM_MIPI_LGIT_CMD_WVGA_INVERSE_PT_PANEL) ||\
			defined(CONFIG_FB_MSM_MIPI_LGIT_VIDEO_WVGA_INVERSE_PT_PANEL)
	.power_on_set = lgit_power_on_set,
	.power_off_set = lgit_power_off_set,
	.power_on_set_size = ARRAY_SIZE(lgit_power_on_set),
	.power_off_set_size = ARRAY_SIZE(lgit_power_off_set),
	.max_backlight_level = 0xFF,
#endif
/* LGE_CHANGE_E LG-4573B H/W Rev.C or upper revision, jamin.koo@lge.com, 2011.02.27 */
#if defined (CONFIG_LGE_BACKLIGHT_LM3530)
	.max_backlight_level = 0x71,
#elif defined (CONFIG_LGE_BACKLIGHT_LM3533)
	.max_backlight_level = 0xFF,
#endif
};

static struct platform_device mipi_dsi_lgit_panel_device = {
	.name = "mipi_lgit",
	.id = 0,
	.dev = {
		.platform_data = &mipi_lgit_pdata,
	}
};

#endif
#endif

#ifdef CONFIG_LGE_KCAL
extern int set_kcal_values(int kcal_r, int kcal_g, int kcal_b);
extern int refresh_kcal_display(void);
extern int get_kcal_values(int *kcal_r, int *kcal_g, int *kcal_b);

static struct kcal_platform_data kcal_pdata = {
	.set_values = set_kcal_values,
	.get_values = get_kcal_values,
	.refresh_display = refresh_kcal_display
};

static struct platform_device kcal_platrom_device = {
	.name   = "kcal_ctrl",
	.dev = {
		.platform_data = &kcal_pdata,
	}
};
#endif
#endif

#ifdef CONFIG_FB_MSM_WRITEBACK_MSM_PANEL
static struct platform_device wfd_panel_device = {
	.name = "wfd_panel",
	.id = 0,
	.dev.platform_data = NULL,
};
static struct platform_device wfd_device = {
	.name          = "msm_wfd",
	.id            = -1,
};
#endif
#ifdef CONFIG_MSM_BUS_SCALING
static struct msm_bus_vectors dtv_bus_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};

static struct msm_bus_vectors dtv_bus_def_vectors[] = {
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 566092800 * 2,
		.ib = 707616000 * 2,
	},
};

static struct msm_bus_paths dtv_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(dtv_bus_init_vectors),
		dtv_bus_init_vectors,
	},
	{
		ARRAY_SIZE(dtv_bus_def_vectors),
		dtv_bus_def_vectors,
	},
};
static struct msm_bus_scale_pdata dtv_bus_scale_pdata = {
	dtv_bus_scale_usecases,
	ARRAY_SIZE(dtv_bus_scale_usecases),
	.name = "dtv",
};

static struct lcdc_platform_data dtv_pdata = {
	.bus_scale_table = &dtv_bus_scale_pdata,
};
#endif

static struct gpiomux_setting mdp_vsync_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting mdp_vsync_active_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct msm_gpiomux_config msm8960_mdp_vsync_configs[] __initdata = {
	{
		.gpio = MDP_VSYNC_GPIO,
		.settings = {
			[GPIOMUX_ACTIVE]    = &mdp_vsync_active_cfg,
			[GPIOMUX_SUSPENDED] = &mdp_vsync_suspend_cfg,
		},
	}
};

#ifdef CONFIG_LGE_HIDDEN_RESET
int lge_get_fb_phys_info(unsigned long *start, unsigned long *size)
{
       if (!start || !size)
               return -1;

       *start = (unsigned long)msm_fb_resources[0].start;
       *size = (unsigned long)(LCD_RESOLUTION_X * LCD_RESOLUTION_Y * 4);

       return 0;
}

void *lge_get_hreset_fb_phys_addr(void)
{
       return (void *)0x88B00000;
}
#endif

static void __init msm_fb_add_devices(void)
{

#ifdef CONFIG_FB_MSM_WRITEBACK_MSM_PANEL
	platform_device_register(&wfd_panel_device);
	platform_device_register(&wfd_device);
#endif

	if (machine_is_msm8x60_rumi3()) {
		msm_fb_register_device("mdp", NULL);
		mipi_dsi_pdata.target_type = 1;
	} else
		msm_fb_register_device("mdp", &mdp_pdata);
	msm_fb_register_device("mipi_dsi", &mipi_dsi_pdata);
#ifdef CONFIG_MSM_BUS_SCALING
	msm_fb_register_device("dtv", &dtv_pdata);
#endif
}

void __init msm8960_allocate_fb_region(void)
{
	void *addr;
	unsigned long size;

	size = MSM_FB_SIZE;
	addr = alloc_bootmem_align(size, 0x1000);
	msm_fb_resources[0].start = __pa(addr);
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for fb\n",
			size, addr, __pa(addr));

}

void __init msm8960_set_display_params(char *prim_panel, char *ext_panel)
{
	if (strnlen(prim_panel, PANEL_NAME_MAX_LEN)) {
		strlcpy(msm_fb_pdata.prim_panel_name, prim_panel,
			PANEL_NAME_MAX_LEN);
		pr_debug("msm_fb_pdata.prim_panel_name %s\n",
			msm_fb_pdata.prim_panel_name);
		if (!strncmp((char *)msm_fb_pdata.prim_panel_name,
			HDMI_PANEL_NAME, strnlen(HDMI_PANEL_NAME,
				PANEL_NAME_MAX_LEN))) {
			pr_debug("HDMI is the primary display by"
				" boot parameter\n");
			hdmi_is_primary = 1;
		}
	}
	if (strnlen(ext_panel, PANEL_NAME_MAX_LEN)) {
		strlcpy(msm_fb_pdata.ext_panel_name, ext_panel,
			PANEL_NAME_MAX_LEN);
		pr_debug("msm_fb_pdata.ext_panel_name %s\n",
			msm_fb_pdata.ext_panel_name);
	}
}

#ifdef CONFIG_I2C

#ifdef CONFIG_LGE_BACKLIGHT_LM3533
#define LM3533_BACKLIGHT_ADDRESS 0x36

struct backlight_platform_data {
   void (*platform_init)(void);
   int gpio;
   unsigned int mode;
   int max_current;
   int init_on_boot;
   int min_brightness;
   int max_brightness;
   int default_brightness;
   int factory_brightness;
};

#if defined(CONFIG_LGE_BACKLIGHT_CABC)
#define PWM_SIMPLE_EN 0xA0
#endif

static struct backlight_platform_data lm3533_data = {
	.gpio = PM8921_GPIO_PM_TO_SYS(24),
	.max_current = 0x13,
	.min_brightness = 0x05,
	.max_brightness = 0xFF,
	.default_brightness = 0x96,
	.factory_brightness = 0x64,
};

static struct i2c_board_info msm_i2c_backlight_info[] = {
	{
		I2C_BOARD_INFO("lm3533", LM3533_BACKLIGHT_ADDRESS),
		.platform_data = &lm3533_data,
	}
};

static struct i2c_registry l_dcm_i2c_backlight_device __initdata = {
		I2C_SURF | I2C_FFA | I2C_FLUID | I2C_RUMI,
		MSM_8960_GSBI2_QUP_I2C_BUS_ID,
		msm_i2c_backlight_info,
		ARRAY_SIZE(msm_i2c_backlight_info),
};
#endif /* CONFIG_LGE_BACKLIGHT_LM3533 */
#endif /* CONFIG_I2C */

static int __init panel_gpiomux_init(void)
{
	int rc;

	rc = msm_gpiomux_init(NR_GPIO_IRQS);
	if (rc == -EPERM) {
		pr_info("%s : msm_gpiomux_init is already initialized\n",
				__func__);
	} else if (rc) {
		pr_err(KERN_ERR "msm_gpiomux_init failed %d\n", rc);
		return rc;
	}

	msm_gpiomux_install(msm8960_mdp_vsync_configs,
			ARRAY_SIZE(msm8960_mdp_vsync_configs));

	return 0;
}

static struct platform_device *l_dcm_panel_devices[] __initdata = {
#ifdef CONFIG_FB_MSM_MIPI_DSI_LGIT
	&mipi_dsi_lgit_panel_device,
#ifdef CONFIG_LGE_LCD_TUNING
	&lcd_misc_device,
#endif
#endif
#ifdef CONFIG_LGE_KCAL
	&kcal_platrom_device,
#endif
};

void __init lge_add_lcd_devices(void)
{
	panel_gpiomux_init();

	fb_register_client(&msm_fb_event_notifier);

	/* LGE_CHANGE_S, Assign command set to panel info as H/W revision, jamin.koo@lge.com, 2011.02.27 */
	if(lge_get_board_revno() < HW_REV_C)
		mipi_dsi_lgit_panel_device.dev.platform_data = &mipi_lgit_pdata_old;
	/* LGE_CHANGE_E, Assign command set to panel info as H/W revision, jamin.koo@lge.com, 2011.02.27 */
	platform_add_devices(l_dcm_panel_devices,
		ARRAY_SIZE(l_dcm_panel_devices));

#ifdef CONFIG_LGE_BACKLIGHT_LM3533
	lge_add_msm_i2c_device(&l_dcm_i2c_backlight_device);
#endif

	msm_fb_add_devices();
	platform_device_register(&msm_fb_device);
}
