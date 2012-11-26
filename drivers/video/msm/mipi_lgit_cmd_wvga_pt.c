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
#include "mipi_lgit.h"
#define DSI_BIT_CLK_300MHZ

static struct msm_panel_info pinfo;

static struct mipi_dsi_phy_ctrl dsi_cmd_mode_phy_db = {
	/* regulator */
	{0x03, 0x0a, 0x04, 0x00, 0x20}, /* Fixed values */
	/* timing */
	{0x66, 0x26, 0x14, 0x00, 0x16, 0x8b, 0x1E, 0x8B, 0x16, 0x03, 0x04, 0xa0},
	/* phy ctrl */
	{0x5f, 0x00, 0x00, 0x10},	/* Fixed values */
	/* strength */
	{0xff, 0x00, 0x06, 0x00},	/* Fixed values */
	/* pll control */
	{0x00, 0x40, 0x01, 0x1a, 0x00, 0x50, 0x48, 0x63,
	0x41, 0x0f, 0x07, 0x00, 0x14, 0x03, 0x00, 0x02,
	0x00, 0x20, 0x00, 0x01 },
};

static int __init mipi_cmd_lgit_wvga_pt_init(void)
{
	int ret;

#ifdef CONFIG_FB_MSM_MIPI_PANEL_DETECT
	if (msm_fb_detect_client("mipi_cmd_lgit_wvga"))
		return 0;
#endif

	pinfo.xres = 480;
	pinfo.yres = 800;
	pinfo.type = MIPI_CMD_PANEL;
	pinfo.pdest = DISPLAY_1;
	pinfo.wait_cycle = 0;
	pinfo.bpp = 24;

	pinfo.lcdc.h_back_porch = 46;
	pinfo.lcdc.h_front_porch = 6;
	pinfo.lcdc.h_pulse_width = 3;
	pinfo.lcdc.v_back_porch = 15;
	pinfo.lcdc.v_front_porch = 10;
	pinfo.lcdc.v_pulse_width = 3;
	pinfo.lcdc.border_clr = 0;
	pinfo.lcdc.underflow_clr = 0xff;
	pinfo.lcdc.hsync_skew = 0;
	pinfo.bl_max = 22;
	pinfo.bl_min = 1;
	pinfo.fb_num = 2;

	pinfo.mipi.te_sel = 1;
	pinfo.mipi.tx_eot_append = 1;
	pinfo.mipi.insert_dcs_cmd = 1;

	pinfo.mipi.mode = DSI_CMD_MODE;
	pinfo.mipi.pulse_mode_hsa_he = FALSE;
	pinfo.mipi.hfp_power_stop = FALSE;
	pinfo.mipi.hbp_power_stop = FALSE;
	pinfo.mipi.hsa_power_stop = FALSE;
	pinfo.mipi.eof_bllp_power_stop = TRUE;
	pinfo.mipi.bllp_power_stop = FALSE;
	pinfo.mipi.traffic_mode = DSI_NON_BURST_SYNCH_EVENT;
	pinfo.mipi.dst_format = DSI_CMD_DST_FORMAT_RGB888;
	pinfo.mipi.vc = 0;
	pinfo.mipi.rgb_swap = DSI_RGB_SWAP_BGR;
	pinfo.mipi.data_lane0 = TRUE;
	pinfo.mipi.data_lane1 = TRUE;
	pinfo.mipi.data_lane2 = FALSE;
	pinfo.mipi.data_lane3 = FALSE;

	pinfo.mipi.t_clk_post = 34;
	pinfo.mipi.t_clk_pre = 51;
	pinfo.mipi.frame_rate = 65;
	pinfo.clk_rate = 300000000;
	pinfo.mipi.stream = 0;
	pinfo.mipi.mdp_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.dma_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.interleave_max = 1;
	pinfo.mipi.wr_mem_continue = 0x3c;
	pinfo.mipi.wr_mem_start = 0x2c;
	pinfo.mipi.dsi_phy_db = &dsi_cmd_mode_phy_db;
	ret = mipi_lgit_device_register(&pinfo, MIPI_DSI_PRIM,
						MIPI_DSI_PANEL_WVGA_PT);
	if (ret)
		printk(KERN_ERR "%s: failed to register device!\n", __func__);

	return ret;
}

module_init(mipi_cmd_lgit_wvga_pt_init);
