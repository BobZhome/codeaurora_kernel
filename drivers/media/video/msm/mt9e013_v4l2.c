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

#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <media/msm_camera.h>
#include <media/v4l2-subdev.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include "mt9e013.h"
#include "msm.h"
/*=============================================================
	SENSOR REGISTER DEFINES
==============================================================*/
#define REG_GROUPED_PARAMETER_HOLD		0x0104
#define GROUPED_PARAMETER_HOLD_OFF		0x00
#define GROUPED_PARAMETER_HOLD			0x01
/* Integration Time */
#define REG_COARSE_INTEGRATION_TIME		0x3012
/* Gain */
#define REG_GLOBAL_GAIN	0x305E
/* PLL registers */
#define REG_FRAME_LENGTH_LINES		0x0340
/* Test Pattern */
#define REG_TEST_PATTERN_MODE			0x0601
#define REG_VCM_CONTROL			0x30F0
#define REG_VCM_NEW_CODE			0x30F2
#define REG_VCM_STEP_TIME			0x30F4

/*============================================================================
							 TYPE DECLARATIONS
============================================================================*/

/* 16bit address - 8 bit context register structure */
#define Q8	0x00000100
#define Q10	0x00000400
#define MT9E013_MASTER_CLK_RATE 24000000

/* AF Total steps parameters */
#define MT9E013_TOTAL_STEPS_NEAR_TO_FAR    37

static uint16_t mt9e013_linear_total_step = MT9E013_TOTAL_STEPS_NEAR_TO_FAR;
static uint16_t mt9e013_step_position_table[MT9E013_TOTAL_STEPS_NEAR_TO_FAR+1];
uint16_t mt9e013_nl_region_boundary1 = 2;
uint16_t mt9e013_nl_region_code_per_step1 = 25;
uint16_t mt9e013_l_region_code_per_step = 3;
uint16_t mt9e013_vcm_step_time;
uint16_t mt9e013_sw_damping_time_wait;


struct mt9e013_work_t {
	struct work_struct work;
};

static struct mt9e013_work_t *mt9e013_sensorw;
static struct i2c_client *mt9e013_client;

struct mt9e013_ctrl_t {
	const struct  msm_camera_sensor_info *sensordata;

	uint32_t sensormode;
	uint32_t fps_divider;/* init to 1 * 0x00000400 */
	uint32_t pict_fps_divider;/* init to 1 * 0x00000400 */
	uint16_t fps;

	uint16_t curr_lens_pos;
	uint16_t curr_step_pos;
	uint16_t my_reg_gain;
	uint32_t my_reg_line_count;
	uint16_t total_lines_per_frame;

	enum mt9e013_resolution_t prev_res;
	enum mt9e013_resolution_t pict_res;
	enum mt9e013_resolution_t curr_res;
	enum mt9e013_test_mode_t  set_test;

	struct v4l2_subdev *sensor_dev;
	struct mt9e013_format *fmt;
};


static bool CSI_CONFIG;
static struct mt9e013_ctrl_t *mt9e013_ctrl;
static DECLARE_WAIT_QUEUE_HEAD(mt9e013_wait_queue);
static DEFINE_MUTEX(mt9e013_mut);

static int cam_debug_init(void);
static struct dentry *debugfs_base;

struct mt9e013_format {
	enum v4l2_mbus_pixelcode code;
	enum v4l2_colorspace colorspace;
	u16 fmt;
	u16 order;
};
/*=============================================================*/

static int mt9e013_i2c_rxdata(unsigned short saddr,
	unsigned char *rxdata, int length)
{
	struct i2c_msg msgs[] = {
		{
			.addr  = saddr,
			.flags = 0,
			.len   = 2,
			.buf   = rxdata,
		},
		{
			.addr  = saddr,
			.flags = I2C_M_RD,
			.len   = 2,
			.buf   = rxdata,
		},
	};
	if (i2c_transfer(mt9e013_client->adapter, msgs, 2) < 0) {
		pr_err("mt9e013_i2c_rxdata faild 0x%x\n", saddr);
		return -EIO;
	}
	return 0;
}

static int32_t mt9e013_i2c_txdata(unsigned short saddr,
				unsigned char *txdata, int length)
{
	struct i2c_msg msg[] = {
		{
			.addr = saddr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		 },
	};
	if (i2c_transfer(mt9e013_client->adapter, msg, 1) < 0) {
		pr_err("mt9e013_i2c_txdata faild 0x%x\n", saddr);
		return -EIO;
	}

	return 0;
}

static int32_t mt9e013_i2c_read(unsigned short raddr,
	unsigned short *rdata, int rlen)
{
	int32_t rc = 0;
	unsigned char buf[2];
	if (!rdata)
		return -EIO;
	memset(buf, 0, sizeof(buf));
	buf[0] = (raddr & 0xFF00) >> 8;
	buf[1] = (raddr & 0x00FF);
	rc = mt9e013_i2c_rxdata(mt9e013_client->addr<<1, buf, rlen);
	if (rc < 0) {
		pr_err("mt9e013_i2c_read 0x%x failed!\n", raddr);
		return rc;
	}
	*rdata = (rlen == 2 ? buf[0] << 8 | buf[1] : buf[0]);
//	pr_err("mt9e013_i2c_read 0x%x val = 0x%x!\n", raddr, *rdata);
	return rc;
}

static int32_t mt9e013_i2c_write_w_sensor(unsigned short waddr, uint16_t wdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[4];
	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00) >> 8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = (wdata & 0xFF00) >> 8;
	buf[3] = (wdata & 0x00FF);
//	pr_err("i2c_write_b addr = 0x%x, val = 0x%x\n", waddr, wdata);
	rc = mt9e013_i2c_txdata(mt9e013_client->addr<<1, buf, 4);
	if (rc < 0) {
		pr_err("i2c_write_b failed, addr = 0x%x, val = 0x%x!\n",
			waddr, wdata);
	}
	return rc;
}

static int32_t mt9e013_i2c_write_b_sensor(unsigned short waddr, uint8_t bdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[3];
	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00) >> 8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = bdata;
//	pr_err("i2c_write_b addr = 0x%x, val = 0x%x\n", waddr, bdata);
	rc = mt9e013_i2c_txdata(mt9e013_client->addr<<1, buf, 3);
	if (rc < 0) {
		pr_err("i2c_write_b failed, addr = 0x%x, val = 0x%x!\n",
			waddr, bdata);
	}
	return rc;
}

static int32_t mt9e013_i2c_write_w_table(struct mt9e013_i2c_reg_conf const
					 *reg_conf_tbl, int num)
{
	int i;
	int32_t rc = -EIO;
	for (i = 0; i < num; i++) {
		rc = mt9e013_i2c_write_w_sensor(reg_conf_tbl->waddr,
			reg_conf_tbl->wdata);
		if (rc < 0)
			break;
		reg_conf_tbl++;
	}
	return rc;
}

static void mt9e013_group_hold_on(void)
{
	mt9e013_i2c_write_b_sensor(REG_GROUPED_PARAMETER_HOLD,
						GROUPED_PARAMETER_HOLD);
}

static void mt9e013_group_hold_off(void)
{
	mt9e013_i2c_write_b_sensor(REG_GROUPED_PARAMETER_HOLD,
						GROUPED_PARAMETER_HOLD_OFF);
}

static void mt9e013_start_stream(void)
{
	mt9e013_i2c_write_w_sensor(0x301A, 0x8250);
	mt9e013_i2c_write_w_sensor(0x301A, 0x8650);
	mt9e013_i2c_write_w_sensor(0x301A, 0x8658);
	mt9e013_i2c_write_b_sensor(0x0104, 0x00);
	mt9e013_i2c_write_w_sensor(0x301A, 0x065C);
}

static void mt9e013_stop_stream(void)
{
	mt9e013_i2c_write_w_sensor(0x301A, 0x0058);
	mt9e013_i2c_write_w_sensor(0x301A, 0x0050);
	mt9e013_i2c_write_b_sensor(0x0104, 0x01);
}

static void mt9e013_get_pict_fps(uint16_t fps, uint16_t *pfps)
{
	/* input fps is preview fps in Q8 format */
	uint32_t divider, d1, d2;

	d1 = mt9e013_regs.reg_prev[E013_FRAME_LENGTH_LINES].wdata
		* 0x00000400/
		mt9e013_regs.reg_snap[E013_FRAME_LENGTH_LINES].wdata;
	d2 = mt9e013_regs.reg_prev[E013_LINE_LENGTH_PCK].wdata
		* 0x00000400/
		mt9e013_regs.reg_snap[E013_LINE_LENGTH_PCK].wdata;
	divider = d1 * d2 / 0x400;

	/*Verify PCLK settings and frame sizes.*/
	*pfps = (uint16_t) (fps * divider / 0x400);
	/* 2 is the ratio of no.of snapshot channels
	to number of preview channels */
}

static uint16_t mt9e013_get_prev_lines_pf(void)
{
	if (mt9e013_ctrl->prev_res == QTR_SIZE)
		return mt9e013_regs.reg_prev[E013_FRAME_LENGTH_LINES].wdata;
	else if (mt9e013_ctrl->prev_res == FULL_SIZE)
		return mt9e013_regs.reg_snap[E013_FRAME_LENGTH_LINES].wdata;
	else if (mt9e013_ctrl->prev_res == HFR_60FPS)
		return mt9e013_regs.reg_60fps[E013_FRAME_LENGTH_LINES].wdata;
	else if (mt9e013_ctrl->prev_res == HFR_90FPS)
		return mt9e013_regs.reg_120fps[E013_FRAME_LENGTH_LINES].wdata;
	else
		return mt9e013_regs.reg_120fps[E013_FRAME_LENGTH_LINES].wdata;
}

static uint16_t mt9e013_get_prev_pixels_pl(void)
{
	if (mt9e013_ctrl->prev_res == QTR_SIZE)
		return mt9e013_regs.reg_prev[E013_LINE_LENGTH_PCK].wdata;
	else if (mt9e013_ctrl->prev_res == FULL_SIZE)
		return mt9e013_regs.reg_snap[E013_LINE_LENGTH_PCK].wdata;
	else if (mt9e013_ctrl->prev_res == HFR_60FPS)
		return mt9e013_regs.reg_60fps[E013_LINE_LENGTH_PCK].wdata;
	else if (mt9e013_ctrl->prev_res == HFR_90FPS)
		return mt9e013_regs.reg_120fps[E013_LINE_LENGTH_PCK].wdata;
	else
		return mt9e013_regs.reg_120fps[E013_LINE_LENGTH_PCK].wdata;
}

static uint16_t mt9e013_get_pict_lines_pf(void)
{
	if (mt9e013_ctrl->pict_res == QTR_SIZE)
		return mt9e013_regs.reg_prev[E013_FRAME_LENGTH_LINES].wdata;
	else if (mt9e013_ctrl->pict_res == FULL_SIZE)
		return mt9e013_regs.reg_snap[E013_FRAME_LENGTH_LINES].wdata;
	else if (mt9e013_ctrl->pict_res == HFR_60FPS)
		return mt9e013_regs.reg_60fps[E013_FRAME_LENGTH_LINES].wdata;
	else if (mt9e013_ctrl->pict_res == HFR_90FPS)
		return mt9e013_regs.reg_120fps[E013_FRAME_LENGTH_LINES].wdata;
	else
		return mt9e013_regs.reg_120fps[E013_FRAME_LENGTH_LINES].wdata;
}

static uint16_t mt9e013_get_pict_pixels_pl(void)
{
	if (mt9e013_ctrl->pict_res == QTR_SIZE)
		return mt9e013_regs.reg_prev[E013_LINE_LENGTH_PCK].wdata;
	else if (mt9e013_ctrl->pict_res == FULL_SIZE)
		return mt9e013_regs.reg_snap[E013_LINE_LENGTH_PCK].wdata;
	else if (mt9e013_ctrl->pict_res == HFR_60FPS)
		return mt9e013_regs.reg_60fps[E013_LINE_LENGTH_PCK].wdata;
	else if (mt9e013_ctrl->pict_res == HFR_90FPS)
		return mt9e013_regs.reg_120fps[E013_LINE_LENGTH_PCK].wdata;
	else
		return mt9e013_regs.reg_120fps[E013_LINE_LENGTH_PCK].wdata;
}

static uint32_t mt9e013_get_pict_max_exp_lc(void)
{
	if (mt9e013_ctrl->pict_res == QTR_SIZE)
		return mt9e013_regs.reg_prev[E013_FRAME_LENGTH_LINES].wdata
			* 24;
	else if (mt9e013_ctrl->pict_res == FULL_SIZE)
		return mt9e013_regs.reg_snap[E013_FRAME_LENGTH_LINES].wdata
			* 24;
	else if (mt9e013_ctrl->pict_res == HFR_60FPS)
		return mt9e013_regs.reg_60fps[E013_FRAME_LENGTH_LINES].wdata
			* 24;
	else if (mt9e013_ctrl->pict_res == HFR_90FPS)
		return mt9e013_regs.reg_120fps[E013_FRAME_LENGTH_LINES].wdata
			* 24;
	else
		return mt9e013_regs.reg_120fps[E013_FRAME_LENGTH_LINES].wdata
			* 24;
}

static int32_t mt9e013_set_fps(struct fps_cfg   *fps)
{
	uint16_t total_lines_per_frame;
	int32_t rc = 0;
	if (mt9e013_ctrl->curr_res == QTR_SIZE)
		total_lines_per_frame =
		mt9e013_regs.reg_prev[E013_FRAME_LENGTH_LINES].wdata;
	else if (mt9e013_ctrl->curr_res == FULL_SIZE)
		total_lines_per_frame =
		mt9e013_regs.reg_snap[E013_FRAME_LENGTH_LINES].wdata;
	else if (mt9e013_ctrl->curr_res == HFR_60FPS)
		total_lines_per_frame =
		mt9e013_regs.reg_60fps[E013_FRAME_LENGTH_LINES].wdata;
	else if (mt9e013_ctrl->curr_res == HFR_90FPS)
		total_lines_per_frame =
		mt9e013_regs.reg_120fps[E013_FRAME_LENGTH_LINES].wdata;
	else
		total_lines_per_frame =
		mt9e013_regs.reg_120fps[E013_FRAME_LENGTH_LINES].wdata;

	mt9e013_ctrl->fps_divider = fps->fps_div;
	mt9e013_ctrl->pict_fps_divider = fps->pict_fps_div;

	if (mt9e013_ctrl->curr_res == FULL_SIZE) {
		total_lines_per_frame = (uint16_t)
		(total_lines_per_frame * mt9e013_ctrl->pict_fps_divider/0x400);
	} else {
		total_lines_per_frame = (uint16_t)
		(total_lines_per_frame * mt9e013_ctrl->fps_divider/0x400);
	}

	mt9e013_group_hold_on();
	rc = mt9e013_i2c_write_w_sensor(REG_FRAME_LENGTH_LINES,
							total_lines_per_frame);
	mt9e013_group_hold_off();
	return rc;
}

static int32_t mt9e013_write_exp_gain(uint16_t gain, uint32_t line)
{
	uint16_t max_legal_gain = 0xE7F;
	int32_t rc = 0;
	if (gain > max_legal_gain) {
		pr_err("Max legal gain Line:%d\n", __LINE__);
		gain = max_legal_gain;
	}

	if (mt9e013_ctrl->curr_res != FULL_SIZE) {
		mt9e013_ctrl->my_reg_gain = gain;
		mt9e013_ctrl->my_reg_line_count = (uint16_t) line;
		line = (uint32_t) (line * mt9e013_ctrl->fps_divider /
						   0x00000400);
	} else {
		line = (uint32_t) (line * mt9e013_ctrl->pict_fps_divider /
						   0x00000400);
	}

	gain |= 0x1000;

	mt9e013_group_hold_on();
	rc = mt9e013_i2c_write_w_sensor(REG_GLOBAL_GAIN, gain);
	rc = mt9e013_i2c_write_w_sensor(REG_COARSE_INTEGRATION_TIME, line);
	mt9e013_group_hold_off();
	return rc;
}

static int32_t mt9e013_set_pict_exp_gain(uint16_t gain, uint32_t line)
{
	int32_t rc = 0;
	rc = mt9e013_write_exp_gain(gain, line);
	mt9e013_i2c_write_w_sensor(0x301A, 0x065C|0x2);
	return rc;
}

static int32_t mt9e013_move_focus(int direction,
	int32_t num_steps)
{
	int16_t step_direction, dest_lens_position, dest_step_position;

	if (direction == MOVE_NEAR)
		step_direction = 1;
	else
		step_direction = -1;

	dest_step_position = mt9e013_ctrl->curr_step_pos
						+ (step_direction * num_steps);

	if (dest_step_position < 0)
		dest_step_position = 0;
	else if (dest_step_position > mt9e013_linear_total_step)
		dest_step_position = mt9e013_linear_total_step;

	if (dest_step_position == mt9e013_ctrl->curr_step_pos)
		return 0;

	CDBG("__debug:MoveFocus, dest_step_position:%d \n", dest_step_position);
	dest_lens_position = mt9e013_step_position_table[dest_step_position];
	if ((dest_step_position <= 4) && (step_direction == 1)) {
		mt9e013_i2c_write_w_sensor(REG_VCM_STEP_TIME, 0x0000);
		if (num_steps == 4) {
			CDBG("__debug:MoveFocus, jumpvalue:%d \n",
			mt9e013_nl_region_boundary1 * mt9e013_nl_region_code_per_step1);
			mt9e013_i2c_write_w_sensor(REG_VCM_NEW_CODE,
			mt9e013_nl_region_boundary1 * mt9e013_nl_region_code_per_step1);
		} else {
			if (dest_step_position <= mt9e013_nl_region_boundary1) {
				CDBG("__debug:MoveFocus, fine search:%d \n",
					dest_lens_position);
				mt9e013_i2c_write_w_sensor(REG_VCM_NEW_CODE,
					dest_lens_position);
				mt9e013_ctrl->curr_lens_pos = dest_lens_position;
				mt9e013_ctrl->curr_step_pos = dest_step_position;
				return 0;
			}
		}
	}

	if(step_direction < 0) {
		if(num_steps > 20) {
			/*macro to infinity*/
			mt9e013_vcm_step_time = 0x0050;
			mt9e013_sw_damping_time_wait = 5;
		} else if (num_steps <= 4) {
			/*reverse search fine step  dir - macro to infinity*/
			mt9e013_vcm_step_time = 0x0400;
			mt9e013_sw_damping_time_wait = 4;
		} else {
			/*reverse search Coarse Jump ( > 4) dir - macro to infinity*/
			mt9e013_vcm_step_time = 0x96;
			mt9e013_sw_damping_time_wait = 3;
			}
	} else {
		if(num_steps >= 4) {
			/*coarse jump  dir - infinity to macro*/
			mt9e013_vcm_step_time = 0x0200;
			mt9e013_sw_damping_time_wait = 2;
		} else {
			/*fine step  dir - infinity to macro*/
			mt9e013_vcm_step_time = 0x0400;
			mt9e013_sw_damping_time_wait = 4;
		}
	}
	mt9e013_i2c_write_w_sensor(REG_VCM_STEP_TIME,
				mt9e013_vcm_step_time);
	CDBG("__debug:MoveFocus, mt9e013_vcm_step_time:%d \n", mt9e013_vcm_step_time);
	CDBG("__debug:MoveFocus, DestLensPosition:%d \n", dest_lens_position);

	if (mt9e013_ctrl->curr_lens_pos != dest_lens_position) {
		mt9e013_i2c_write_w_sensor(REG_VCM_NEW_CODE,
		dest_lens_position);
		usleep(mt9e013_sw_damping_time_wait * 1000);
	}
	mt9e013_ctrl->curr_lens_pos = dest_lens_position;
	mt9e013_ctrl->curr_step_pos = dest_step_position;
	return 0;
}

static int32_t mt9e013_set_default_focus(uint8_t af_step)
{
	int32_t rc = 0;
	if (mt9e013_ctrl->curr_step_pos != 0) {
		rc = mt9e013_move_focus(MOVE_FAR,
		mt9e013_ctrl->curr_step_pos);
	} else {
		mt9e013_i2c_write_w_sensor(REG_VCM_NEW_CODE, 0x00);
	}

	mt9e013_ctrl->curr_lens_pos = 0;
	mt9e013_ctrl->curr_step_pos = 0;

	return rc;
}

static void mt9e013_init_focus(void)
{
	uint8_t i;
	mt9e013_step_position_table[0] = 0;
	for (i = 1; i <= mt9e013_linear_total_step; i++) {
		if (i <= mt9e013_nl_region_boundary1) {
			mt9e013_step_position_table[i] =
				mt9e013_step_position_table[i-1]
				+ mt9e013_nl_region_code_per_step1;
		} else {
			mt9e013_step_position_table[i] =
				mt9e013_step_position_table[i-1]
				+ mt9e013_l_region_code_per_step;
		}

		if (mt9e013_step_position_table[i] > 255)
			mt9e013_step_position_table[i] = 255;
	}
	mt9e013_ctrl->curr_lens_pos = 0;
}

static int32_t mt9e013_test(enum mt9e013_test_mode_t mo)
{
	int32_t rc = 0;
	if (mo == TEST_OFF)
		return rc;
	else {
		/* REG_0x30D8[4] is TESBYPEN: 0: Normal Operation,
		1: Bypass Signal Processing
		REG_0x30D8[5] is EBDMASK: 0:
		Output Embedded data, 1: No output embedded data */
		if (mt9e013_i2c_write_b_sensor(REG_TEST_PATTERN_MODE,
			(uint8_t) mo) < 0) {
			return rc;
		}
	}
	return rc;
}

static int32_t mt9e013_sensor_setting(int update_type, int rt)
{
	int32_t rc = 0;

	pr_err("%s: update_type = %d, rt = %d\n", __func__, update_type, rt);

	mt9e013_stop_stream();
	msleep(15);
	if (update_type == REG_INIT) {
		mt9e013_i2c_write_w_table(mt9e013_regs.reg_mipi,
			mt9e013_regs.reg_mipi_size);
		mt9e013_i2c_write_w_table(mt9e013_regs.rec_settings,
			mt9e013_regs.rec_size);
		cam_debug_init();
		CSI_CONFIG = 0;
	} else if (update_type == UPDATE_PERIODIC) {
		//msleep(100);
		if (!CSI_CONFIG) {
			struct msm_camera_csid_params mt9e013_csid_params;
			struct msm_camera_csiphy_params mt9e013_csiphy_params;
			struct msm_camera_csid_vc_cfg mt9e013_vccfg[] = {
				{0, CSI_RAW10, CSI_DECODE_10BIT},
				{1, CSI_EMBED_DATA, CSI_DECODE_10BIT},
			};
			mt9e013_csid_params.lane_cnt = 2;
			mt9e013_csid_params.lane_assign = 0xe4;
			mt9e013_csid_params.lut_params.num_cid =
				ARRAY_SIZE(mt9e013_vccfg);
			mt9e013_csid_params.lut_params.vc_cfg =
				&mt9e013_vccfg[0];
			mt9e013_csiphy_params.lane_cnt = 2;
			mt9e013_csiphy_params.settle_cnt = 0x1B;
			v4l2_subdev_notify(s_ctrl->sensor_v4l2_subdev,
					NOTIFY_CSID_CFG,
					&mt9e013_csid_params);
			v4l2_subdev_notify(mt9e013_ctrl->sensor_dev,
					NOTIFY_CID_CHANGE, NULL);
			dsb();
			v4l2_subdev_notify(s_ctrl->sensor_v4l2_subdev,
					NOTIFY_CSIPHY_CFG,
					&mt9e013_csiphy_params);
			dsb();
			msleep(10);
			CSI_CONFIG = 1;
		}
		if (rt == QTR_SIZE) {
			mt9e013_i2c_write_w_table(mt9e013_regs.reg_pll,
				mt9e013_regs.reg_pll_size);
			mt9e013_i2c_write_w_table(mt9e013_regs.reg_prev,
				mt9e013_regs.reg_prev_size);
		} else if (rt == FULL_SIZE) {
			mt9e013_i2c_write_w_table(mt9e013_regs.reg_pll,
				mt9e013_regs.reg_pll_size);
			mt9e013_i2c_write_w_table(mt9e013_regs.reg_snap,
				mt9e013_regs.reg_snap_size);
		} else if (rt == HFR_60FPS) {
			mt9e013_i2c_write_w_table(mt9e013_regs.reg_pll_120fps,
				mt9e013_regs.reg_pll_120fps_size);
			mt9e013_i2c_write_w_sensor(0x0306, 0x0029);
			mt9e013_i2c_write_w_table(mt9e013_regs.reg_120fps,
				mt9e013_regs.reg_120fps_size);
		} else if (rt == HFR_90FPS) {
			mt9e013_i2c_write_w_table(mt9e013_regs.reg_pll_120fps,
				mt9e013_regs.reg_pll_120fps_size);
			mt9e013_i2c_write_w_sensor(0x0306, 0x003D);
			mt9e013_i2c_write_w_table(mt9e013_regs.reg_120fps,
				mt9e013_regs.reg_120fps_size);
		} else if (rt == HFR_120FPS) {
			msm_camio_vfe_clk_rate_set(266667000);
			mt9e013_i2c_write_w_table(mt9e013_regs.reg_pll_120fps,
				mt9e013_regs.reg_pll_120fps_size);
			mt9e013_i2c_write_w_table(mt9e013_regs.reg_120fps,
				mt9e013_regs.reg_120fps_size);
		}

		mt9e013_start_stream();
	}
	return rc;
}

static int32_t mt9e013_video_config(int mode)
{

	int32_t rc = 0;

	pr_err("%s: Start: mode = %d\n", __func__, mode);

	/* change sensor resolution if needed */
	if (mt9e013_sensor_setting(UPDATE_PERIODIC,
			mt9e013_ctrl->prev_res) < 0)
		return rc;
	if (mt9e013_ctrl->set_test) {
		if (mt9e013_test(mt9e013_ctrl->set_test) < 0)
			return  rc;
	}

	mt9e013_ctrl->curr_res = mt9e013_ctrl->prev_res;
	mt9e013_ctrl->sensormode = mode;
	return rc;
}

static int32_t mt9e013_snapshot_config(int mode)
{
	int32_t rc = 0;

	pr_err("%s: Start: mode = %d\n", __func__, mode);

	/*change sensor resolution if needed */
	if (mt9e013_ctrl->curr_res != mt9e013_ctrl->pict_res) {
		if (mt9e013_sensor_setting(UPDATE_PERIODIC,
				mt9e013_ctrl->pict_res) < 0)
			return rc;
	}

	mt9e013_ctrl->curr_res = mt9e013_ctrl->pict_res;
	mt9e013_ctrl->sensormode = mode;
	return rc;
}

static int32_t mt9e013_raw_snapshot_config(int mode)
{
	int32_t rc = 0;

	pr_err("%s: Start: mode = %d\n", __func__, mode);

	/* change sensor resolution if needed */
	if (mt9e013_ctrl->curr_res != mt9e013_ctrl->pict_res) {
		if (mt9e013_sensor_setting(UPDATE_PERIODIC,
				mt9e013_ctrl->pict_res) < 0)
			return rc;
	}

	mt9e013_ctrl->curr_res = mt9e013_ctrl->pict_res;
	mt9e013_ctrl->sensormode = mode;
	return rc;
}

static int32_t mt9e013_set_sensor_mode(int mode,
	int res)
{
	int32_t rc = 0;

	pr_err("%s: mode = %d, res = %d", __func__, mode, res);

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
	case SENSOR_HFR_60FPS_MODE:
	case SENSOR_HFR_90FPS_MODE:
	case SENSOR_HFR_120FPS_MODE:
		mt9e013_ctrl->prev_res = res;
		rc = mt9e013_video_config(mode);
		break;
	case SENSOR_SNAPSHOT_MODE:
		mt9e013_ctrl->pict_res = res;
		rc = mt9e013_snapshot_config(mode);
		break;
	case SENSOR_RAW_SNAPSHOT_MODE:
		mt9e013_ctrl->pict_res = res;
		rc = mt9e013_raw_snapshot_config(mode);
		break;
	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}

static int32_t mt9e013_af_power_down(void)
{
	if (mt9e013_ctrl->curr_lens_pos != 0)
	{
		mt9e013_set_default_focus(0);
		msleep(40);
	}
	mt9e013_i2c_write_w_sensor(REG_VCM_CONTROL, 0x00);
	return 0;
}

static int32_t mt9e013_power_down(void)
{
	mt9e013_af_power_down();
	return 0;
}

static int mt9e013_probe_init_done(const struct msm_camera_sensor_info *data)
{
	gpio_set_value_cansleep(data->sensor_reset, 0);
	gpio_direction_input(data->sensor_reset);
	gpio_free(data->sensor_reset);
	return 0;
}

static int mt9e013_probe_init_sensor(const struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;
	uint16_t chipid = 0;

	pr_info("%s: Entered!\n", __func__);

	/* Reset */
	rc = gpio_request(data->sensor_reset, "mt9e013");
	if (!rc) {
		rc = gpio_direction_output(data->sensor_reset, 0);
		pr_info("%s: sensor_reset = %d, rc = %d\n", __func__, data->sensor_reset, rc);
		msleep(10);
		gpio_set_value_cansleep(data->sensor_reset, 1);
		msleep(10);
	} else {
		goto probe_init_sensor_fail;
	}

	/* Compare sensor ID to MT9E013 ID: 0x4B00 */
	rc = mt9e013_i2c_read(0x0000, &chipid, 2);
	pr_err("Sensor ID: %d(0x%x)\n", chipid, chipid);
	if (chipid != 0x4B00) {
		rc = -ENODEV;
		pr_err("mt9e013_probe_init_sensor fail chip id doesnot match\n");
		goto probe_init_sensor_fail;
	}

	pr_err(" mt9e013_probe_init_sensor finishes\n");
	return rc;

probe_init_sensor_fail:
	pr_err(" mt9e013_probe_init_sensor fails\n");
	mt9e013_probe_init_done(data);
	return rc;
}

static int mt9e013_sensor_open_init(const struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;

	pr_info("%s: Entered!\n", __func__);

	mt9e013_ctrl->fps_divider = 1 * 0x00000400;
	mt9e013_ctrl->pict_fps_divider = 1 * 0x00000400;
	mt9e013_ctrl->fps = 30*Q8;
	mt9e013_ctrl->set_test = TEST_OFF;
	mt9e013_ctrl->prev_res = QTR_SIZE;
	mt9e013_ctrl->pict_res = FULL_SIZE;

	if (data)
		mt9e013_ctrl->sensordata = data;

	/* Enable MCLK first */
	msm_camio_clk_rate_set(MT9E013_MASTER_CLK_RATE);

	rc = mt9e013_probe_init_sensor(data);
	if (rc < 0)
		goto sensor_open_init_fail;

	pr_err("%s: init settings\n", __func__);
	rc = mt9e013_sensor_setting(REG_INIT, mt9e013_ctrl->prev_res);
	if (rc < 0)
		goto sensor_open_init_fail;

	mt9e013_init_focus();

	pr_err("mt9e013_sensor_open_init done\n");
	return rc;

sensor_open_init_fail:
	pr_err("mt9e013_sensor_open_init fail\n");
	mt9e013_probe_init_done(data);
	return rc;
}

static int mt9e013_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&mt9e013_wait_queue);
	return 0;
}

static const struct i2c_device_id mt9e013_i2c_id[] = {
	{"mt9e013", 0},
	{ }
};

static int mt9e013_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	pr_err("mt9e013_i2c_probe called!\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		rc = -ENOTSUPP;
		pr_err("i2c_check_functionality failed\n");
		goto i2c_probe_failure;
	}

	mt9e013_sensorw = kzalloc(sizeof(struct mt9e013_work_t), GFP_KERNEL);
	if (!mt9e013_sensorw) {
		rc = -ENOMEM;
		pr_err("kzalloc failed.\n");
		goto i2c_probe_failure;
	}

	i2c_set_clientdata(client, mt9e013_sensorw);
	mt9e013_init_client(client);
	mt9e013_client = client;

	pr_err("mt9e013_i2c_probe successed! rc = %d\n", rc);
	return 0;

i2c_probe_failure:
	if( mt9e013_sensorw ) kfree(mt9e013_sensorw);
	mt9e013_sensorw = NULL;
	pr_err("mt9e013_i2c_probe failed! rc = %d\n", rc);
	return rc;
}

static int mt9e013_send_wb_info(struct wb_info_cfg *wb)
{
	return 0;
}

static int __exit mt9e013_remove(struct i2c_client *client)
{
	struct mt9e013_work_t_t *sensorw = i2c_get_clientdata(client);
	free_irq(client->irq, sensorw);
	mt9e013_client = NULL;
	kfree(sensorw);
	return 0;
}

static struct i2c_driver mt9e013_i2c_driver = {
	.id_table = mt9e013_i2c_id,
	.probe  = mt9e013_i2c_probe,
	.remove = __exit_p(mt9e013_i2c_remove),
	.driver = {
		.name = "mt9e013",
	},
};

static int mt9e013_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cdata;
	long   rc = 0;

	if (copy_from_user(&cdata,
		(void *)argp,
		sizeof(struct sensor_cfg_data)))
		return -EFAULT;

	mutex_lock(&mt9e013_mut);
	pr_err("%s: cfgtype = %d\n", __func__, cdata.cfgtype);

	switch (cdata.cfgtype) {
	case CFG_GET_PICT_FPS:
		mt9e013_get_pict_fps(
			cdata.cfg.gfps.prevfps,
			&(cdata.cfg.gfps.pictfps));

		if (copy_to_user((void *)argp,
			&cdata,
			sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PREV_L_PF:
		cdata.cfg.prevl_pf =
		mt9e013_get_prev_lines_pf();

		if (copy_to_user((void *)argp,
			&cdata,
			sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PREV_P_PL:
		cdata.cfg.prevp_pl =
			mt9e013_get_prev_pixels_pl();

		if (copy_to_user((void *)argp,
			&cdata,
			sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PICT_L_PF:
		cdata.cfg.pictl_pf =
			mt9e013_get_pict_lines_pf();

		if (copy_to_user((void *)argp,
			&cdata,
			sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PICT_P_PL:
		cdata.cfg.pictp_pl =
			mt9e013_get_pict_pixels_pl();

		if (copy_to_user((void *)argp,
			&cdata,
			sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PICT_MAX_EXP_LC:
		cdata.cfg.pict_max_exp_lc = mt9e013_get_pict_max_exp_lc();

		if (copy_to_user((void *)argp,
			&cdata,
			sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_SET_FPS:
	case CFG_SET_PICT_FPS:
		rc = mt9e013_set_fps(&(cdata.cfg.fps));
		break;

	case CFG_SET_EXP_GAIN:
		rc = mt9e013_write_exp_gain(cdata.cfg.exp_gain.gain,
				cdata.cfg.exp_gain.line);
		break;

	case CFG_SET_PICT_EXP_GAIN:
		rc = mt9e013_set_pict_exp_gain(cdata.cfg.exp_gain.gain,
			cdata.cfg.exp_gain.line);
		break;

	case CFG_SET_MODE:
		rc = mt9e013_set_sensor_mode(cdata.mode,
				cdata.rs);
		break;

	case CFG_PWR_DOWN:
		rc = mt9e013_power_down();
		break;

	case CFG_MOVE_FOCUS:
		rc = mt9e013_move_focus(cdata.cfg.focus.dir,
			cdata.cfg.focus.steps);
		break;

	case CFG_SET_DEFAULT_FOCUS:
		rc = mt9e013_set_default_focus(cdata.cfg.focus.steps);
		break;

	case CFG_GET_AF_MAX_STEPS:
		cdata.max_steps = mt9e013_linear_total_step;
		if (copy_to_user((void *)argp,
			&cdata,
			sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_SET_EFFECT:
		rc = mt9e013_set_default_focus(cdata.cfg.effect);
		break;


	case CFG_SEND_WB_INFO:
		rc = mt9e013_send_wb_info(&(cdata.cfg.wb_info));
		break;

	default:
		rc = -EFAULT;
		break;
	}

	mutex_unlock(&mt9e013_mut);

	return rc;
}

static int mt9e013_sensor_release(void)
{
	int rc = -EBADF;

	mutex_lock(&mt9e013_mut);
	mt9e013_power_down();
	gpio_set_value_cansleep(mt9e013_ctrl->sensordata->sensor_reset, 0);
	msleep(5);
	gpio_free(mt9e013_ctrl->sensordata->sensor_reset);
	mutex_unlock(&mt9e013_mut);
	pr_err("%s: mt9e013_release completed\n", __func__);

	return rc;
}

static int mt9e013_sensor_probe(const struct msm_camera_sensor_info *info,
		struct msm_sensor_ctrl *s)
{
	int rc = 0;
	rc = i2c_add_driver(&mt9e013_i2c_driver);
	if (rc < 0 || mt9e013_client == NULL) {
		rc = -ENOTSUPP;
		pr_err("I2C add driver failed");
		goto probe_fail;
	}

	/* Set clock */
	msm_camio_clk_rate_set(MT9E013_MASTER_CLK_RATE);

	/* Reset sensor and probe sensor registers thru I2C */
	rc = mt9e013_probe_init_sensor(info);
	if (rc < 0)
		goto probe_fail;

	s->s_init = mt9e013_sensor_open_init;
	s->s_release = mt9e013_sensor_release;
	s->s_config  = mt9e013_sensor_config;
	s->s_camera_type = BACK_CAMERA_2D;
	s->s_mount_angle = info->sensor_platform_info->mount_angle;

	mt9e013_probe_init_done(info);

	pr_info("mt9e013_sensor_probe : SUCCESS!\n");
	return rc;

probe_fail:
	pr_err("mt9e013_sensor_probe: SENSOR PROBE FAILS!\n");
	return rc;
}

static struct mt9e013_format mt9e013_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order  = 0,
	},
	/* more can be supported, to be added later */
};

static int mt9e013_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
			   enum v4l2_mbus_pixelcode *code)
{
	printk(KERN_DEBUG "Index is %d\n", index);
	if ((unsigned int)index >= ARRAY_SIZE(mt9e013_subdev_info))
		return -EINVAL;

	*code = mt9e013_subdev_info[index].code;
	return 0;
}

static struct v4l2_subdev_core_ops mt9e013_subdev_core_ops;
static struct v4l2_subdev_video_ops mt9e013_subdev_video_ops = {
	.enum_mbus_fmt = mt9e013_enum_fmt,
};

static struct v4l2_subdev_ops mt9e013_subdev_ops = {
	.core = &mt9e013_subdev_core_ops,
	.video  = &mt9e013_subdev_video_ops,
};

static int mt9e013_sensor_probe_cb(const struct msm_camera_sensor_info *info,
	struct v4l2_subdev *sdev, struct msm_sensor_ctrl *s)
{
	int rc = 0;

	mt9e013_ctrl = kzalloc(sizeof(struct mt9e013_ctrl_t), GFP_KERNEL);
	if (!mt9e013_ctrl) {
		pr_err("%s: kzalloc failed!\n", __func__);
		return -ENOMEM;
	}

	rc = mt9e013_sensor_probe(info, s);
	if (rc < 0) {
		pr_err("%s: mt9m114_sensor_probe fail\n", __func__);
		return rc;
	}

	/* probe is successful, init a v4l2 subdevice */
	printk(KERN_DEBUG "going into v4l2_i2c_subdev_init\n");
	if (sdev) {
		v4l2_i2c_subdev_init(sdev, mt9e013_client,
						&mt9e013_subdev_ops);
		mt9e013_ctrl->sensor_dev = sdev;
	}
	return rc;
}

static int __mt9e013_probe(struct platform_device *pdev)
{
	pr_err("####### __mt9e013_probe()");
	return msm_sensor_register(pdev, mt9e013_sensor_probe_cb);
}

static struct platform_driver msm_camera_driver = {
	.probe = __mt9e013_probe,
	.driver = {
		.name = "msm_camera_mt9e013",
		.owner = THIS_MODULE,
	},
};

static int __init mt9e013_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

module_init(mt9e013_init);
MODULE_DESCRIPTION("Aptina 8 MP Bayer sensor driver");
MODULE_LICENSE("GPL v2");

static bool streaming = 1;

static int mt9e013_set_af_codestep(void *data, u64 val)
{
	mt9e013_l_region_code_per_step = val;
	mt9e013_init_focus();
	return 0;
}

static int mt9e013_get_af_codestep(void *data, u64 *val)
{
	*val = mt9e013_l_region_code_per_step;
	return 0;
}

static int mt9e013_set_linear_total_step(void *data, u64 val)
{
	mt9e013_linear_total_step = val;
	return 0;
}

static int mt9e013_af_linearity_test(void *data, u64 *val)
{
	int i = 0;
	mt9e013_set_default_focus(0);
	msleep(3000);
	for (i = 0; i < mt9e013_linear_total_step; i++) {
		mt9e013_move_focus(MOVE_NEAR, 1);
		pr_err("__debug:MOVE_NEAR moved to index =[%d]\n", i);
	msleep(1000);
	}
	for (i = 0; i < mt9e013_linear_total_step; i++) {
		mt9e013_move_focus(MOVE_FAR, 1);
		CDBG("__debug:MOVE_FAR moved to index =[%d]\n", i);
		msleep(1000);
	}
	return 0;
}
static uint16_t mt9e013_step_jump = 4;
static uint8_t mt9e013_step_dir = MOVE_NEAR;
static int mt9e013_af_step_config(void *data, u64 val)
{
	mt9e013_step_jump = val & 0xFFFF;
	mt9e013_step_dir = (val >> 16) & 0x1;
	return 0;
}

static int mt9e013_af_step(void *data, u64 *val)
{
	int i = 0;
	int dir = MOVE_NEAR;
	mt9e013_set_default_focus(0);
	if (mt9e013_step_dir == 1)
			dir = MOVE_FAR;

	for (i = 1; i < MT9E013_TOTAL_STEPS_NEAR_TO_FAR; i+=mt9e013_step_jump) {
		mt9e013_move_focus(dir, mt9e013_step_jump);
		msleep(1000);
	}
	mt9e013_set_default_focus(0);
	return 0;
}
static int mt9e013_af_set_slew(void *data, u64 val)
{
	mt9e013_vcm_step_time = val & 0xFFFF;
	return 0;
}

static int mt9e013_af_get_slew(void *data, u64 *val)
{
	*val = mt9e013_vcm_step_time;
	return 0;
}

static int mt9e013_set_sw_damping(void *data, u64 val)
{
	mt9e013_sw_damping_time_wait = val;
	return 0;
}

static int mt9e013_get_sw_damping(void *data, u64 *val)
{
	*val = mt9e013_sw_damping_time_wait;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(af_damping, mt9e013_get_sw_damping,
			mt9e013_set_sw_damping, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(af_codeperstep, mt9e013_get_af_codestep,
	mt9e013_set_af_codestep, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(af_linear, mt9e013_af_linearity_test,
	mt9e013_set_linear_total_step, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(af_step, mt9e013_af_step,
	mt9e013_af_step_config, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(af_slew, mt9e013_af_get_slew,
	mt9e013_af_set_slew, "%llu\n");

static int cam_debug_stream_set(void *data, u64 val)
{
	int rc = 0;

	if (val) {
		mt9e013_start_stream();
		streaming = 1;
	} else {
		mt9e013_stop_stream();
		streaming = 0;
	}

	return rc;
}

static int cam_debug_stream_get(void *data, u64 *val)
{
	*val = streaming;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(cam_stream, cam_debug_stream_get,
			cam_debug_stream_set, "%llu\n");


static int cam_debug_init(void)
{
	struct dentry *cam_dir;
	debugfs_base = debugfs_create_dir("sensor", NULL);
	if (!debugfs_base)
		return -ENOMEM;

	cam_dir = debugfs_create_dir("mt9e013", debugfs_base);
	if (!cam_dir)
		return -ENOMEM;
	if (!debugfs_create_file("af_codeperstep", S_IRUGO | S_IWUSR, cam_dir,
			NULL, &af_codeperstep))
			return -ENOMEM;
	if (!debugfs_create_file("af_linear", S_IRUGO | S_IWUSR, cam_dir,
			NULL, &af_linear))
		return -ENOMEM;
	if (!debugfs_create_file("af_step", S_IRUGO | S_IWUSR, cam_dir,
			NULL, &af_step))
		return -ENOMEM;
	if (!debugfs_create_file("af_slew", S_IRUGO | S_IWUSR, cam_dir,
			NULL, &af_slew))
		return -ENOMEM;
	if (!debugfs_create_file("af_damping", S_IRUGO | S_IWUSR, cam_dir,
			NULL, &af_damping))
		return -ENOMEM;
	if (!debugfs_create_file("stream", S_IRUGO | S_IWUSR, cam_dir,
							 NULL, &cam_stream))
		return -ENOMEM;

	return 0;
}

