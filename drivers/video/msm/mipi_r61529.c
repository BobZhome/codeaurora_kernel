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
 */

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_r61529.h"
#include <asm/gpio.h>
#include <mach/vreg.h>
#include <mach/board_lge.h>

static void mipi_ldp_lcd_panel_poweroff(void);

static struct msm_panel_common_pdata *mipi_r61529_pdata;

static struct dsi_buf r61529_tx_buf;
static struct dsi_buf r61529_rx_buf;

/*---------------------- display_on ----------------------------*/
static char disp_sleep_out[1] = {0x11};
static char disp_display_on[1] = {0x29};

static struct dsi_cmd_desc r61529_disp_on_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 40,
		sizeof(disp_display_on), disp_display_on}
};

static struct dsi_cmd_desc r61529_sleep_out_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 120,
		sizeof(disp_sleep_out), disp_sleep_out},
};

static int mipi_r61529_lcd_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct mipi_panel_info *mipi;

	mfd = platform_get_drvdata(pdev);
	mipi = &mfd->panel_info.mipi;

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	mipi_dsi_cmds_tx(mfd, &r61529_tx_buf, r61529_sleep_out_cmds,
			ARRAY_SIZE(r61529_sleep_out_cmds));

	mipi_dsi_cmds_tx(mfd, &r61529_tx_buf, mipi_r61529_pdata->power_on_set,
			mipi_r61529_pdata->power_on_set_size);

	mipi_dsi_cmds_tx(mfd, &r61529_tx_buf, r61529_disp_on_cmds,
			ARRAY_SIZE(r61529_disp_on_cmds));

	return 0;
}

static int mipi_r61529_lcd_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	mipi_dsi_cmds_tx(mfd, &r61529_tx_buf,
			mipi_r61529_pdata->power_off_set,
			mipi_r61529_pdata->power_off_set_size);

	mipi_ldp_lcd_panel_poweroff();

	return 0;
}

static void mipi_r61529_set_backlight_board(struct msm_fb_data_type *mfd)
{
	int level;

	level = (int)mfd->bl_level;
	mipi_r61529_pdata->backlight_level(level, 0, 0);
}

ssize_t mipi_r61529_lcd_show_onoff(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("%s : start\n", __func__);
	return 0;
}

ssize_t mipi_r61529_lcd_store_onoff(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	/*struct platform_device dummy_pdev;*/
	int onoff;
	struct platform_device *pdev = to_platform_device(dev);

	sscanf(buf, "%d", &onoff);
	printk("%s: onoff : %d\n", __func__, onoff);

	if (onoff) {
		mipi_r61529_lcd_on(pdev);
	} else {
		mipi_r61529_lcd_off(pdev);
	}
	return count;
}

DEVICE_ATTR(lcd_onoff, 0664, mipi_r61529_lcd_show_onoff, mipi_r61529_lcd_store_onoff);


static int __devinit mipi_r61529_lcd_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct msm_fb_panel_data *pdata;

	if (pdev->id == 0) {
		mipi_r61529_pdata = pdev->dev.platform_data;
		return 0;
	}

	pdata = pdev->dev.platform_data;
	if (!pdata)
		return 0;

	pdata->panel_info.bl_max = mipi_r61529_pdata->max_backlight_level;

	msm_fb_add_device(pdev);
	/*this for AT Command*/
	rc = device_create_file(&pdev->dev, &dev_attr_lcd_onoff);

	return 0;
}

static struct platform_driver this_driver = {
	.probe  = mipi_r61529_lcd_probe,
	.driver = {
		.name   = "mipi_r61529",
	},
};

static struct msm_fb_panel_data r61529_panel_data = {
	.on		= mipi_r61529_lcd_on,
	.off	= mipi_r61529_lcd_off,
	.set_backlight = mipi_r61529_set_backlight_board,
};

static int ch_used[3];

int mipi_r61529_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	int ret;

	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	ch_used[channel] = TRUE;

	pdev = platform_device_alloc("mipi_r61529", (panel << 8)|channel);
	if (!pdev)
		return -ENOMEM;

	r61529_panel_data.panel_info = *pinfo;

	ret = platform_device_add_data(pdev, &r61529_panel_data,
		sizeof(r61529_panel_data));
	if (ret) {
		pr_err("%s: platform_device_add_data failed!\n", __func__);
		goto err_device_put;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		pr_err("%s: platform_device_register failed!\n", __func__);
		goto err_device_put;
	}

	return 0;

err_device_put:
	platform_device_put(pdev);
	return ret;
}

static int __init mipi_r61529_lcd_init(void)
{
	mipi_dsi_buf_alloc(&r61529_tx_buf, DSI_BUF_SIZE);
	mipi_dsi_buf_alloc(&r61529_rx_buf, DSI_BUF_SIZE);

	return platform_driver_register(&this_driver);
}

static void mipi_ldp_lcd_panel_poweroff(void)
{
	/* stay GPIO_LCD_RESET high in deep sleep*/
	/*gpio_set_value(GPIO_LCD_RESET, 0);*/
	mdelay(10);
}

module_init(mipi_r61529_lcd_init);
