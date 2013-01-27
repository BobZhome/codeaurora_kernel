/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
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
#include "mipi_hitachi.h"

static struct msm_panel_info pinfo;

#if defined (CONFIG_MACH_MSM8960_D1LV)
/* To be out from GSM band */
#define DSI_BIT_CLK_482MHZ
#else
#define DSI_BIT_CLK_370MHZ
#endif

static struct mipi_dsi_phy_ctrl dsi_video_mode_phy_db = {
	/* 720*1280, RGB888, 4 Lane 60 fps video mode */
#if defined(DSI_BIT_CLK_370MHZ)
    /* regulator */
#if defined(CONFIG_MACH_MSM8960_D1L_KR)
	#if defined (CONFIG_MACH_MSM8960_D1LU)
		{0x03, 0x0a, 0x04, 0x00, 0x20},	/* Fixed values */		
	#else
		{0x05, 0x0a, 0x04, 0x00, 0x20},	/* DSI1_DSIPHY_REGULATOR_CTRL_0 :  0x1(0.415V) -> 0x11(0.385V) */
	#endif
#else
	{0x03, 0x0a, 0x04, 0x00, 0x20},	/* Fixed values */
#endif
	/* timing */
	{0x66, 0x26, 0x0E, 0x00, 0x10, 0x85, 0x1E, 0x88,
	0x10, 0x03, 0x04},
    /* phy ctrl */
	{0x5f, 0x00, 0x00, 0x10},	/* Fixed values */
    /* strength */
	{0xff, 0x00, 0x06, 0x00},	/* Fixed values */
	/* pll control */
	{0x00, 0xD4, 0x01, 0x1a, 0x00, 0x50, 0x48, 0x63,
	0x41, 0x0f, 0x03, 0x00, 0x14, 0x03, 0x00, 0x02,
	0x00, 0x20, 0x00, 0x01 },
#elif defined(DSI_BIT_CLK_482MHZ)
    /* regulator */
	{0x03, 0x0a, 0x04, 0x00, 0x20},	/* Fixed values */
	/* timing */
	{0x66, 0x26, 0x1F, 0x00, 0x22, 0x97, 0x1E, 0x8F,
	0x22, 0x03, 0x04},
    /* phy ctrl */
	{0x5f, 0x00, 0x00, 0x10},	/* Fixed values */
    /* strength */
	{0xff, 0x00, 0x06, 0x00},	/* Fixed values */
	/* pll control */
	{0x00, 0xe1, 0x01, 0x1a, 0x00, 0x50, 0x48, 0x63,
	0x41, 0x0f, 0x03, 0x00, 0x14, 0x03, 0x00, 0x02,
	0x00, 0x20, 0x00, 0x01 },
#endif


};

static int __init mipi_video_hitachi_hd_pt_init(void)
{
	int ret;

#ifdef CONFIG_FB_MSM_MIPI_PANEL_DETECT
	if (msm_fb_detect_client("mipi_video_hitachi_hd"))
		return 0;
#endif

	pinfo.xres = 768;		//ori 720
	pinfo.yres = 1024;	//ori 1280
	pinfo.type = MIPI_VIDEO_PANEL;
	pinfo.pdest = DISPLAY_1;
	pinfo.wait_cycle = 0;
	pinfo.bpp = 24;

	/* QCT Limitation :
	 * All proch values must be a multiple of 4. 2011.01.20 */
#if defined(DSI_BIT_CLK_370MHZ)
	pinfo.lcdc.h_back_porch = 60;
	pinfo.lcdc.h_front_porch = 110;
	pinfo.lcdc.h_pulse_width = 2;
	pinfo.lcdc.v_back_porch = 5;
	pinfo.lcdc.v_front_porch = 20;
	pinfo.lcdc.v_pulse_width = 2;
#elif defined(DSI_BIT_CLK_482MHZ)
	pinfo.lcdc.h_back_porch = 192;
	pinfo.lcdc.h_front_porch = 108;
	pinfo.lcdc.h_pulse_width = 4;
	pinfo.lcdc.v_back_porch = 4;
	pinfo.lcdc.v_front_porch = 23;
	pinfo.lcdc.v_pulse_width = 1;
#endif



	pinfo.lcdc.border_clr = 0;	/* blk */
	pinfo.lcdc.underflow_clr = 0xff;	/* blue */
	pinfo.lcdc.hsync_skew = 0;
	pinfo.bl_max = 0x7F;
	pinfo.bl_min = 0;
	pinfo.fb_num = 2;

	pinfo.mipi.mode = DSI_VIDEO_MODE;
	pinfo.mipi.pulse_mode_hsa_he = FALSE;
	pinfo.mipi.hfp_power_stop = TRUE;
	pinfo.mipi.hbp_power_stop = TRUE;
	pinfo.mipi.hsa_power_stop = TRUE;
	pinfo.mipi.eof_bllp_power_stop = TRUE;
	pinfo.mipi.bllp_power_stop = TRUE;
	pinfo.mipi.traffic_mode = DSI_BURST_MODE; 
	pinfo.mipi.dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
	pinfo.mipi.vc = 0;
	pinfo.mipi.rgb_swap = DSI_RGB_SWAP_BGR;
	pinfo.mipi.data_lane0 = TRUE;
	pinfo.mipi.data_lane1 = TRUE;
	pinfo.mipi.data_lane2 = TRUE;
	pinfo.mipi.data_lane3 = TRUE;
#if defined(DSI_BIT_CLK_370MHZ)
	pinfo.mipi.t_clk_post = 0x22;
	pinfo.mipi.t_clk_pre = 0x34;
	pinfo.clk_rate = 234580000;
	pinfo.mipi.frame_rate = 40;
#elif defined(DSI_BIT_CLK_482MHZ)
	pinfo.mipi.t_clk_post = 0x22;
	pinfo.mipi.t_clk_pre = 0x36;
	pinfo.clk_rate = 482180000;
	pinfo.mipi.frame_rate = 60;
#endif




	pinfo.mipi.stream = 0; /* dma_p */
	pinfo.mipi.mdp_trigger = DSI_CMD_TRIGGER_SW;/* DSI_CMD_TRIGGER_SW; */
	pinfo.mipi.dma_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.dsi_phy_db = &dsi_video_mode_phy_db;
	ret = mipi_hitachi_device_register(&pinfo, MIPI_DSI_PRIM,
						MIPI_DSI_PANEL_WVGA_PT);
	if (ret)
		printk(KERN_ERR "%s: failed to register device!\n", __func__);

	return ret;
}

module_init(mipi_video_hitachi_hd_pt_init);
