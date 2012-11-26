/* Copyright (c) 2008-2010, Code Aurora Forum. All rights reserved.
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_lgit.h"

#if defined(CONFIG_FB_MSM_MIPI_LGIT_VIDEO_WVGA_INVERSE_PT_PANEL)
#include <mach/gpio.h>
#include <mach/board_lge.h>
#include CONFIG_BOARD_HEADER_FILE
#endif

#if defined (CONFIG_LGE_BACKLIGHT_CABC) && \
	defined (CONFIG_LGE_BACKLIGHT_CABC_DEBUG)
extern void set_lgit_cabc(int idx);
extern int get_lgit_cabc(void);
#endif

static struct msm_panel_common_pdata *mipi_lgit_pdata;

static struct dsi_buf lgit_tx_buf;
static struct dsi_buf lgit_rx_buf;

/* LGE_CHANGE
* Add code to use several LCD panel.
* 2011-06-30, minjong.gong@lge.com
*/
#ifdef CONFIG_FB_MSM_MIPI_LGIT_VIDEO_HD_PT /*LG display 4.5" HD for D1L */
#define LGIT_IEF

/* minjong.gong@lge.com 2011.03.22,  Modify code to apply IEF function */
static char dsi_config[6] = {0xE0, 0x43, 0x00, 0x80, 0x00, 0x00};
static char display_mode1[6] = {0xB5, 0x14, 0x20, 0x40, 0x00, 0x00};
static char display_mode2[6] = {0xB6, 0x01, 0x16, 0x0F, 0x16, 0x13};

static char p_gamma_r_setting[10] = {0xD0, 0x00, 0x66, 0x76, 0x04, 0x02, 0x02,
	0x42, 0x02, 0x03};
static char n_gamma_r_setting[10] = {0xD1, 0x00, 0x66, 0x76, 0x04, 0x02, 0x02,
	0x42, 0x02, 0x03};
static char p_gamma_g_setting[10] = {0xD2, 0x00, 0x66, 0x76, 0x04, 0x02, 0x02,
	0x42, 0x02, 0x03};
static char n_gamma_g_setting[10] = {0xD3, 0x00, 0x66, 0x76, 0x04, 0x02, 0x02,
	0x42, 0x02, 0x03};
static char p_gamma_b_setting[10] = {0xD4, 0x00, 0x66, 0x76, 0x04, 0x02, 0x02,
	0x42, 0x02, 0x03};
static char n_gamma_b_setting[10] = {0xD5, 0x00, 0x66, 0x76, 0x04, 0x02, 0x02,
	0x42, 0x02, 0x03};

#if defined(LGIT_IEF)
static char ief_set0[2] = {0x70, 0x07};
static char ief_set1[5] = {0x71, 0x00, 0x00, 0x01, 0x01};
static char ief_set2[3] = {0x72, 0x01, 0x0F};
static char ief_set3[4] = {0x73, 0x34, 0x66, 0x40};
static char ief_set4[4] = {0x74, 0x04, 0x0F, 0x02};
static char ief_set5[4] = {0x75, 0x03, 0x8F, 0x07};
static char ief_set6[4] = {0x76, 0x07, 0x00, 0x05};
#endif

static char osc_setting[3] =     {0xC0, 0x00, 0x00};
/*shoogi.lee@lge.com 2011.03.31, Modify code to apply final vlaue*/
static char power_setting3[10] = {0xC3, 0x01, 0x08, 0x00, 0x00, 0x00,
	0x67, 0x88, 0x32, 0x02};
static char power_setting4[6] =  {0xC4, 0x22, 0x24, 0x19, 0x19, 0x41};
static char otp2_setting[2] =    {0XF9, 0x00};

static char exit_sleep[2] =  {0x11, 0x00};
static char display_on[2] =  {0x29, 0x00};
static char enter_sleep[2] = {0x10, 0x00};
static char display_off[2] = {0x28, 0x00};
static char deep_standby[2] = {0xC1, 0x01};

/* initialize device */
static struct dsi_cmd_desc lgit_power_on_set[] = {
	/* Display Initial Set */
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(dsi_config), dsi_config},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(display_mode1), display_mode1},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(display_mode2), display_mode2},

	/* Gamma Set */
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(p_gamma_r_setting),
		p_gamma_r_setting},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(n_gamma_r_setting),
		n_gamma_r_setting},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(p_gamma_g_setting),
		p_gamma_g_setting},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(n_gamma_g_setting),
		n_gamma_g_setting},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(p_gamma_b_setting),
		p_gamma_b_setting},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(n_gamma_b_setting),
		n_gamma_b_setting},

#if defined(LGIT_IEF)
	/* Image Enhancement Function Set */
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(ief_set0), ief_set0},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(ief_set1), ief_set1},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(ief_set2), ief_set2},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(ief_set3), ief_set3},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(ief_set4), ief_set4},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(ief_set5), ief_set5},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(ief_set6), ief_set6},
#endif

	/* Power Supply Set */
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(osc_setting), osc_setting},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(power_setting3), power_setting3},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(power_setting4), power_setting4},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 10, sizeof(otp2_setting), otp2_setting},
	{DTYPE_DCS_WRITE,  1, 0, 0, 150, sizeof(exit_sleep), exit_sleep},
	{DTYPE_DCS_WRITE,  1, 0, 0, 50, sizeof(display_on), display_on},
};

static struct dsi_cmd_desc lgit_power_off_set[] = {
	/* wait 3frames */
	{DTYPE_DCS_WRITE, 1, 0, 0, 100, sizeof(display_off), display_off},
	/* wait 7frames */
	{DTYPE_DCS_WRITE, 1, 0, 0, 150, sizeof(enter_sleep), enter_sleep},
	{DTYPE_DCS_WRITE, 1, 0, 0, 0, sizeof(deep_standby), deep_standby},
};

#elif defined(CONFIG_FB_MSM_MIPI_LGIT_CMD_WVGA_PT)
/* LG Display 4.3" WVGA for B2L */
static char inversion_off[2] = {0x20, 0x00};
static char mem_access_add[5] = {0x2B, 0x00, 0x00, 0x03, 0x1F};
static char tear_on[2] = {0x35, 0x00};
static char if_pixel_format[2] = {0x3A, 0x77};
static char panel_setting[3] = {0xB2, 0x00, 0xC7};	/* 0xCA -> 0xC7 */
static char drive_setting[2] = {0xB3, 0x00};	/* Column Inversion */

static char display_ctrl1[8] = {0xB5, 0x20, 0x10, 0x10, 0x00, 0x20, 0x00, 0x00};
static char display_ctrl2[9] = {0xB6, 0x00, 0x08, 0x02, 0x40, 0x20, 0x20, 0x00,
	0x00};
static char display_ctrl3[8] = {0xB7, 0x52, 0x00, 0x10, 0x00, 0x0B, 0x00, 0x00};

static char osc_setting[4] = {0xC0, 0x01, 0x0B, 0x0F};

static char power_ctrl2[3] = {0xC2, 0x07, 0x00};
static char power_ctrl3[6] = {0xC3, 0x20, 0x67, 0x00, 0x08, 0x08};
static char power_ctrl4[7] = {0xC4, 0x01, 0x17, 0x00, 0x00, 0x33, 0x08};
static char power_ctrl5[6] = {0xC5, 0x12, 0x33, 0x14, 0x14, 0x13};
static char power_ctrl6[3] = {0xC6, 0x22, 0x00};
static char power_ctrl7[6] = {0xC7, 0x04, 0x1F, 0x1F, 0x68, 0x00};
static char power_ctrl8[3] = {0xC8, 0x44, 0x63};

static char p_gamma_r_setting[10] = {0xD0, 0x00, 0x16, 0x62, 0x35, 0x02, 0x00,
	0x30, 0x00, 0x03};
static char n_gamma_r_setting[10] = {0xD1, 0x00, 0x16, 0x62, 0x35, 0x02, 0x00,
	0x30, 0x00, 0x03};
static char p_gamma_g_setting[10] = {0xD2, 0x00, 0x16, 0x62, 0x35, 0x02, 0x00,
	0x30, 0x00, 0x03};
static char n_gamma_g_setting[10] = {0xD3, 0x00, 0x16, 0x62, 0x35, 0x02, 0x00,
	0x30, 0x00, 0x03};
static char p_gamma_b_setting[10] = {0xD4, 0x00, 0x16, 0x62, 0x35, 0x02, 0x00,
	0x30, 0x00, 0x03};
static char n_gamma_b_setting[10] = {0xD5, 0x00, 0x16, 0x62, 0x35, 0x02, 0x00,
	0x30, 0x00, 0x03};

static char exit_sleep[2] =  {0x11, 0x00};
static char display_on[2] =  {0x29, 0x00};
static char enter_sleep[2] = {0x10, 0x00};
static char display_off[2] = {0x28, 0x00};
static char deep_standby[2] = {0xC1, 0x01};

/* initialize device */
static struct dsi_cmd_desc lgit_power_on_set[] = {
	{DTYPE_DCS_WRITE,  1, 0, 0, 0, sizeof(inversion_off), inversion_off},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(mem_access_add), mem_access_add},
	{DTYPE_DCS_WRITE,  1, 0, 0, 0, sizeof(tear_on), tear_on},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(if_pixel_format),
		if_pixel_format},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(panel_setting), panel_setting},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(drive_setting), drive_setting},

	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(display_ctrl1), display_ctrl1},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(display_ctrl2), display_ctrl2},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(display_ctrl3), display_ctrl3},

	{DTYPE_DCS_LWRITE, 1, 0, 0, 5, sizeof(osc_setting), osc_setting},

	{DTYPE_DCS_LWRITE, 1, 0, 0,   0, sizeof(power_ctrl2), power_ctrl2},
	{DTYPE_DCS_LWRITE, 1, 0, 0,   0, sizeof(power_ctrl3), power_ctrl3},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 100, sizeof(power_ctrl4), power_ctrl4},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 100, sizeof(power_ctrl5), power_ctrl5},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 100, sizeof(power_ctrl6), power_ctrl6},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 100, sizeof(power_ctrl7), power_ctrl7},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 100, sizeof(power_ctrl8), power_ctrl8},

	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(p_gamma_r_setting),
		p_gamma_r_setting},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(n_gamma_r_setting),
		n_gamma_r_setting},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(p_gamma_g_setting),
		p_gamma_g_setting},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(n_gamma_g_setting),
		n_gamma_g_setting},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(p_gamma_b_setting),
		p_gamma_b_setting},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(n_gamma_b_setting),
		n_gamma_b_setting},

	{DTYPE_DCS_WRITE,  1, 0, 0, 100, sizeof(exit_sleep), exit_sleep},
	{DTYPE_DCS_WRITE,  1, 0, 0, 100, sizeof(display_on), display_on},
};

static struct dsi_cmd_desc lgit_power_off_set[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 100, sizeof(display_off), display_off},
	{DTYPE_DCS_WRITE, 1, 0, 0, 100, sizeof(enter_sleep), enter_sleep},
	{DTYPE_DCS_WRITE, 1, 0, 0, 0,   sizeof(deep_standby), deep_standby},
};

#elif defined(CONFIG_FB_MSM_MIPI_LGIT_VIDEO_WVGA_PT)
/* LG Display 4.3" for B2L(Video Mode) */
static char inversion_off[2] = {0x20, 0x00};
static char tear_on[2] = {0x35, 0x00};
static char if_pixel_format[2] = {0x3A, 0x77};
static char panel_setting[3] = {0xB2, 0x00, 0xC7};
static char drive_setting[2] = {0xB3, 0x02};

static char display_ctrl1[8] = {0xB5, 0x29, 0x10, 0x10, 0x00, 0x20, 0x00, 0x00};
static char display_ctrl2[9] = {0xB6, 0x00, 0x18, 0x02, 0x40, 0x20, 0x20, 0x05,
	0x05};
static char display_ctrl3[8] = {0xB7, 0x52, 0x00, 0x10, 0x00, 0x0B, 0x00, 0x00};

static char osc_setting[4] = {0xC0, 0x00, 0x0B, 0x0F};

static char power_ctrl2[3] = {0xC2, 0x07, 0x00};
static char power_ctrl3[6] = {0xC3, 0x20, 0x67, 0x00, 0x08, 0x08};
static char power_ctrl4[7] = {0xC4, 0x11, 0x17, 0x00, 0x00, 0x22, 0x08};
static char power_ctrl5[6] = {0xC5, 0x12, 0x33, 0x14, 0x14, 0x13};
static char power_ctrl6[3] = {0xC6, 0x22, 0x00};
static char power_ctrl7[6] = {0xC7, 0x04, 0x1F, 0x1F, 0x5f, 0x00};
static char power_ctrl8[3] = {0xC8, 0x44, 0x63};

static char vref_gen[4] = {0xCB, 0x07, 0x15, 0x34};
static char vodeo_mode[6] = {0xE0, 0x43, 0x40, 0x80, 0x00, 0x00};

static char p_gamma_r_setting[10] = {0xD0, 0x01, 0x46, 0x77, 0x03, 0x00, 0x00,
	0x03, 0x00, 0x01};
static char n_gamma_r_setting[10] = {0xD1, 0x01, 0x46, 0x77, 0x03, 0x00, 0x00,
	0x03, 0x00, 0x01};
static char p_gamma_g_setting[10] = {0xD2, 0x01, 0x46, 0x77, 0x03, 0x00, 0x00,
	0x03, 0x00, 0x01};
static char n_gamma_g_setting[10] = {0xD3, 0x01, 0x46, 0x77, 0x03, 0x00, 0x00,
	0x03, 0x00, 0x01};
static char p_gamma_b_setting[10] = {0xD4, 0x01, 0x46, 0x77, 0x03, 0x00, 0x00,
	0x03, 0x00, 0x01};
static char n_gamma_b_setting[10] = {0xD5, 0x01, 0x46, 0x77, 0x03, 0x00, 0x00,
	0x03, 0x00, 0x01};

static char enter_sleep[2] 	= {0x10, 0x00};
static char exit_sleep[2] 	= {0x11, 0x00};
static char display_off[2] 	= {0x28, 0x00};
static char display_on[2] 	= {0x29, 0x00};
static char deep_standby[2] = {0xC1, 0x01};

/* initialize device */
static struct dsi_cmd_desc lgit_power_on_set[] = {
	{DTYPE_DCS_WRITE,  1, 0, 0, 0, sizeof(inversion_off), inversion_off},
	{DTYPE_DCS_WRITE,  1, 0, 0, 0, sizeof(tear_on), tear_on},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(if_pixel_format),	if_pixel_format},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(panel_setting), panel_setting},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(drive_setting), drive_setting},

	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(display_ctrl1), display_ctrl1},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(display_ctrl2), display_ctrl2},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(display_ctrl3), display_ctrl3},

	{DTYPE_DCS_LWRITE, 1, 0, 0, 5, sizeof(osc_setting), osc_setting},

	{DTYPE_DCS_LWRITE, 1, 0, 0,   0, sizeof(power_ctrl2), power_ctrl2},
	{DTYPE_DCS_LWRITE, 1, 0, 0,   0, sizeof(power_ctrl3), power_ctrl3},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 100, sizeof(power_ctrl4), power_ctrl4},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 100, sizeof(power_ctrl5), power_ctrl5},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 100, sizeof(power_ctrl6), power_ctrl6},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 100, sizeof(power_ctrl7), power_ctrl7},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 100, sizeof(power_ctrl8), power_ctrl8},

	{DTYPE_DCS_LWRITE, 1, 0, 0, 100, sizeof(vref_gen), vref_gen},
	{DTYPE_DCS_LWRITE, 1, 0, 0,   0, sizeof(vodeo_mode), vodeo_mode},

	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(p_gamma_r_setting),
		p_gamma_r_setting},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(n_gamma_r_setting),
		n_gamma_r_setting},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(p_gamma_g_setting),
		p_gamma_g_setting},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(n_gamma_g_setting),
		n_gamma_g_setting},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(p_gamma_b_setting),
		p_gamma_b_setting},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(n_gamma_b_setting),
		n_gamma_b_setting},

	{DTYPE_DCS_WRITE,  1, 0, 0, 100, sizeof(exit_sleep), exit_sleep},
	{DTYPE_DCS_WRITE,  1, 0, 0, 100, sizeof(display_on), display_on},
};

static struct dsi_cmd_desc lgit_power_off_set[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 100, sizeof(display_off), display_off},
	{DTYPE_DCS_WRITE, 1, 0, 0, 100, sizeof(enter_sleep), enter_sleep},
	{DTYPE_DCS_WRITE, 1, 0, 0, 0,   sizeof(deep_standby), deep_standby},
};
#endif
static int mipi_lgit_lcd_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	
#if defined(CONFIG_FB_MSM_MIPI_LGIT_VIDEO_WVGA_INVERSE_PT_PANEL)
	static int gpio43 = PM8921_GPIO_PM_TO_SYS(43);
#endif /* CONFIG_FB_MSM_MIPI_LGIT_VIDEO_WVGA_INVERSE_PT_PANEL */

	mfd = platform_get_drvdata(pdev);
	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	printk(KERN_INFO "%s: mipi lgit lcd on started \n", __func__);

	/* LGE_CHANGE_S, jamin.koo@lge.com, Change the Power On Sequence of LCD, 2012.03.28*/
#if defined(CONFIG_FB_MSM_MIPI_LGIT_VIDEO_WVGA_INVERSE_PT_PANEL)
	gpio_direction_output(gpio43, 0);
	mdelay(12);
	gpio_direction_output(gpio43, 1);
#endif /* CONFIG_FB_MSM_MIPI_LGIT_VIDEO_WVGA_INVERSE_PT_PANEL */
	/* LGE_CHANGE_E, jamin.koo@lge.com, Change the Power On Sequence of LCD, 2012.03.28*/

	mipi_dsi_cmds_tx(mfd, &lgit_tx_buf, mipi_lgit_pdata->power_on_set,
			mipi_lgit_pdata->power_on_set_size);

	return 0;
}

static int mipi_lgit_lcd_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	mfd = platform_get_drvdata(pdev);
	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	printk(KERN_INFO "%s: mipi lgit lcd off started \n", __func__);
	mipi_dsi_cmds_tx(mfd, &lgit_tx_buf, mipi_lgit_pdata->power_off_set,
			mipi_lgit_pdata->power_off_set_size);

	return 0;
}

static void mipi_lgit_set_backlight_board(struct msm_fb_data_type *mfd)
{
	int level;

	level = (int)mfd->bl_level;

	mipi_lgit_pdata->backlight_level(level, 0x05, 0xFF);
}

static int mipi_lgit_lcd_probe(struct platform_device *pdev)
{
	if (pdev->id == 0) {
		mipi_lgit_pdata = pdev->dev.platform_data;
		return 0;
	}

	printk(KERN_INFO "%s: mipi lgit lcd probe start\n", __func__);

	msm_fb_add_device(pdev);

	return 0;
}

static struct platform_driver this_driver = {
	.probe  = mipi_lgit_lcd_probe,
	.driver = {
		.name   = "mipi_lgit",
	},
};

static struct msm_fb_panel_data lgit_panel_data = {
	.on		= mipi_lgit_lcd_on,
	.off		= mipi_lgit_lcd_off,
	.set_backlight = mipi_lgit_set_backlight_board,
};

static int ch_used[3];

#if defined (CONFIG_LGE_BACKLIGHT_CABC) && \
	defined (CONFIG_LGE_BACKLIGHT_CABC_DEBUG)
static ssize_t lgit_cabc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	get_lgit_cabc();
	return sprintf(buf, "%d\n", get_lgit_cabc());
}

static ssize_t lgit_cabc_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int cabc_index;

	if (!count)
		return -EINVAL;

	cabc_index = simple_strtoul(buf, NULL, 10);

	set_lgit_cabc(cabc_index);

	return count;

}
DEVICE_ATTR(lgit_cabc, 0664, lgit_cabc_show, lgit_cabc_store);
#endif

int mipi_lgit_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	int ret;

	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	ch_used[channel] = TRUE;

	pdev = platform_device_alloc("mipi_lgit", (panel << 8)|channel);
	if (!pdev)
		return -ENOMEM;

	lgit_panel_data.panel_info = *pinfo;

	ret = platform_device_add_data(pdev, &lgit_panel_data,
		sizeof(lgit_panel_data));
	if (ret) {
		printk(KERN_ERR
		  "%s: platform_device_add_data failed!\n", __func__);
		goto err_device_put;
	}

	ret = platform_device_add(pdev);

	if (ret) {
		printk(KERN_ERR
		  "%s: platform_device_register failed!\n", __func__);
		goto err_device_put;
	}

#if defined (CONFIG_LGE_BACKLIGHT_CABC) && \
	defined (CONFIG_LGE_BACKLIGHT_CABC_DEBUG)
	ret = device_create_file(&pdev->dev, &dev_attr_lgit_cabc);
	if (ret) {
		printk(KERN_ERR
		  "%s: device_create_file failed!\n", __func__);
		goto err_device_put;
	}
#endif

	

	return 0;

err_device_put:
	platform_device_put(pdev);
	return ret;
}

static int __init mipi_lgit_lcd_init(void)
{
	mipi_dsi_buf_alloc(&lgit_tx_buf, DSI_BUF_SIZE);
	mipi_dsi_buf_alloc(&lgit_rx_buf, DSI_BUF_SIZE);

	return platform_driver_register(&this_driver);
}

module_init(mipi_lgit_lcd_init);
