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

static struct msm_panel_info pinfo;

// G1TDR_MPCS_FRW [hoseok.kim@lge.com 20120402] Apply QCT Case #00807552 patch to fix Frame Rate and DSI clock.
// change PLL 1 ~ 3 (0xf9, 0xb0, 0xda).
static struct mipi_dsi_phy_ctrl dsi_video_mode_phy_db = {
	/* regulator */
	{0x03, 0x0a, 0x04, 0x00, 0x20}, /* Fixed values */
	/* timing */
	{0x66, 0x26, 0x1a, 0x00, 0x1d, 0x91, 0x1e,
	 0x8d, 0x1d, 0x03, 0x04},
	/* phy ctrl */
	{0x5f, 0x00, 0x00, 0x10},	/* Fixed values */
	/* strength */
	{0xff, 0x00, 0x06, 0x00},	/* Fixed values */
	/* pll control */
	/* [CAUTION] The 11th value must be calculated according to pll settings */
	{0x00, 0xf9, 0xb0, 0xda, 0x00, 0x20, 0x48, 0x63,
	 0x41, 0x0f, 0x07,
	 0x00, 0x14, 0x03, 0x00, 0x02, 0x00, 0x20, 0x00, 0x01},
};

static int __init mipi_video_r61529_hvga_pt_init(void)
{
	int ret;

#ifdef CONFIG_FB_MSM_MIPI_PANEL_DETECT
	if (msm_fb_detect_client("mipi_video_r61529_hvga"))
		return 0;
#endif

	pinfo.xres = 320;
	pinfo.yres = 480;
	pinfo.type = MIPI_VIDEO_PANEL;
	pinfo.pdest = DISPLAY_1;
	pinfo.wait_cycle = 0;
	pinfo.bpp = 24;

	pinfo.lcdc.h_back_porch = 40;
	pinfo.lcdc.h_front_porch = 100;
	pinfo.lcdc.h_pulse_width = 1;
	pinfo.lcdc.v_back_porch = 6;
	pinfo.lcdc.v_front_porch = 6;
	pinfo.lcdc.v_pulse_width = 2;

	pinfo.clk_rate = 334080000;

	pinfo.lcdc.border_clr = 0;	/* blk */
	pinfo.lcdc.underflow_clr = 0xff;	/* blue */
	pinfo.lcdc.hsync_skew = 0;
	pinfo.bl_max = 0x71;
	pinfo.bl_min = 1;
	pinfo.fb_num = 2;

	pinfo.mipi.mode = DSI_VIDEO_MODE;
	pinfo.mipi.pulse_mode_hsa_he = TRUE;
	pinfo.mipi.hfp_power_stop = TRUE;
	pinfo.mipi.hbp_power_stop = TRUE;
	pinfo.mipi.hsa_power_stop = TRUE;
	pinfo.mipi.eof_bllp_power_stop = TRUE;
	pinfo.mipi.bllp_power_stop = TRUE;

	pinfo.mipi.traffic_mode = DSI_NON_BURST_SYNCH_EVENT; /*DSI_BURST_MODE*/
	pinfo.mipi.dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
	pinfo.mipi.vc = 0;
	pinfo.mipi.rgb_swap = DSI_RGB_SWAP_RGB;
	pinfo.mipi.data_lane0 = TRUE;
	pinfo.mipi.data_lane1 = FALSE;

	pinfo.mipi.t_clk_post = 0x22;
	pinfo.mipi.t_clk_pre = 0x33;

	pinfo.mipi.stream = 0; /* dma_p */
	pinfo.mipi.mdp_trigger = DSI_CMD_TRIGGER_NONE;
	pinfo.mipi.dma_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.frame_rate = 60;
	pinfo.mipi.dsi_phy_db = &dsi_video_mode_phy_db;
	pinfo.mipi.dlane_swap = 0x00;
	pinfo.mipi.tx_eot_append = 0x01;

	ret = mipi_r61529_device_register(&pinfo, MIPI_DSI_PRIM,
						MIPI_DSI_PANEL_FWVGA_PT);
	if (ret)
		pr_err("%s: failed to register device!\n", __func__);

	return ret;
}

module_init(mipi_video_r61529_hvga_pt_init);
