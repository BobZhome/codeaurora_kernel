/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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

#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <media/msm_camera.h>
#include <media/v4l2-subdev.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include "mt9m114.h"
#include "msm.h"

/* Micron MT9M114 Registers and their values */

#define SENSOR_DEBUG 0

/*============================================================================
							 TYPE DECLARATIONS
============================================================================*/

/* 16bit address - 8 bit context register structure */
#define Q8	0x00000100
#define Q10	0x00000400
#define MT9M114_MASTER_CLK_RATE 24000000

struct mt9m114_work {
	struct work_struct work;
};

static struct mt9m114_work *mt9m114_sensorw;
static struct i2c_client *mt9m114_client;

struct mt9m114_ctrl_t {
	const struct msm_camera_sensor_info *sensordata;

	uint32_t sensormode;
	uint32_t fps_divider;/* init to 1 * 0x00000400 */
	uint32_t pict_fps_divider;/* init to 1 * 0x00000400 */
	uint16_t fps;

	enum mt9m114_resolution_t prev_res;
	enum mt9m114_resolution_t pict_res;
	enum mt9m114_resolution_t curr_res;
	enum mt9m114_test_mode_t  set_test;

	struct v4l2_subdev *sensor_dev;
	struct mt9m114_format *fmt;
};

/*
static uint8_t mt9m114_delay_msecs_stdby = 5;
static uint16_t mt9m114_delay_msecs_stream = 5;
*/
static int32_t config_csi;

static struct mt9m114_ctrl_t *mt9m114_ctrl;
static DECLARE_WAIT_QUEUE_HEAD(mt9m114_wait_queue);
static DEFINE_MUTEX(mt9m114_mut);

static int prev_effect_mode;
static int prev_balance_mode;

struct mt9m114_format {
	enum v4l2_mbus_pixelcode code;
	enum v4l2_colorspace colorspace;
	u16 fmt;
	u16 order;
};

/*=============================================================
	EXTERNAL DECLARATIONS
==============================================================*/
extern struct mt9m114_reg mt9m114_regs;


/*=============================================================*/

static int mt9m114_i2c_rxdata(unsigned short saddr,
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
			.len   = length,
			.buf   = rxdata,
		},
	};
	if (i2c_transfer(mt9m114_client->adapter, msgs, 2) < 0) {
		pr_err("mt9m114_i2c_rxdata faild 0x%x\n", saddr);
		return -EIO;
	}
	return 0;
}

static int32_t mt9m114_i2c_txdata(unsigned short saddr,
				unsigned char *txdata, int length)
{
	struct i2c_msg msg[] = {
		{
			.addr  = saddr,
			.flags = 0,
			.len   = length,
			.buf   = txdata,
		},
	};
	if (i2c_transfer(mt9m114_client->adapter, msg, 1) < 0) {
		pr_err("mt9m114_i2c_txdata failed: saddr = 0x%x\n", saddr);
		return -EIO;
	}

	return 0;
}

static int32_t mt9m114_i2c_read(unsigned short raddr,
	unsigned short *rdata, int rlen)
{
	int32_t rc = 0;
	unsigned char buf[2];
	if (!rdata)
		return -EIO;
	memset(buf, 0, sizeof(buf));
	buf[0] = (raddr & 0xFF00) >> 8;
	buf[1] = (raddr & 0x00FF);
	rc = mt9m114_i2c_rxdata(mt9m114_client->addr<<1, buf, rlen);
	if (rc < 0) {
		pr_err("mt9m114_i2c_read 0x%x failed!\n", raddr);
		return rc;
	}
	*rdata = (rlen == 2 ? buf[0] << 8 | buf[1] : buf[0]);
	CDBG("mt9m114_i2c_read 0x%x val = 0x%x!\n", raddr, *rdata);
	return rc;
}

static int32_t mt9m114_i2c_write_w_sensor(unsigned short waddr, uint16_t wdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[4];
	memset(buf, 0, sizeof(buf));

	buf[0] = (waddr & 0xFF00) >> 8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = (wdata & 0xFF00) >> 8;
	buf[3] = (wdata & 0x00FF);

	rc = mt9m114_i2c_txdata(mt9m114_client->addr<<1, buf, 4);
	if (rc < 0) {
		pr_err("i2c_write_w failed, addr = 0x%x, val = 0x%x!\n",
			waddr, wdata);
	}

	CDBG("i2c_write_w addr = 0x%x, val = 0x%x, rc = %d\n", waddr, wdata, rc);
	return rc;
}

static int32_t mt9m114_i2c_write_table(struct mt9m114_i2c_reg_conf const *reg_conf_tbl, int num_of_items_in_table)
{
	int i;
	int32_t rc = -EIO;

	for (i = 0; i < num_of_items_in_table; i++) {
		if (reg_conf_tbl->waddr == 0xFFFF) {
			msleep(reg_conf_tbl->wdata);
			rc = 0;
		} else if (reg_conf_tbl->waddr == 0xFFFE) {
			unsigned short test_data = 0;
			for (i = 0; i < 50; i++) {
				/* max delay ==> 500 ms */
				rc  = mt9m114_i2c_read(0x0080, &test_data, WORD_LEN);
				if (rc < 0)
					return rc;


				if((test_data & reg_conf_tbl->wdata)==0)
					break;
				else
					mdelay(10);

/*				printk(KERN_ERR "### %s :  Polling set, 0x0080 Reg : 0x%x\n", __func__, test_data);  */
			}
		} else if (reg_conf_tbl->waddr == 0x301A) {
			unsigned short test_data = 0;
			rc  = mt9m114_i2c_read(0x301A, &test_data, WORD_LEN);
			if (rc < 0)
				return rc;

			rc = mt9m114_i2c_write_w_sensor(0x301A, test_data|0x0200);
			if (rc < 0)
				return rc;

/*			printk(KERN_ERR "### %s : Reset reg check, 0x301A Reg : 0x%x\n", __func__, test_data|0x0200);  */
		} else if ((reg_conf_tbl->waddr == 0x0080) && ((reg_conf_tbl->wdata == 0x8000) || (reg_conf_tbl->wdata == 0x0001))) {
			unsigned short test_data = 0;
			rc  = mt9m114_i2c_read(0x0080, &test_data, WORD_LEN);
			if (rc < 0)
				return rc;

			test_data = test_data|reg_conf_tbl->wdata;
			rc = mt9m114_i2c_write_w_sensor(0x0080, test_data);
			if (rc < 0)
				return rc;

/*			printk(KERN_ERR "### %s : Patch check, 0x0080 Reg : 0x%x\n", __func__, test_data);  */
		} else {
			rc = mt9m114_i2c_write_w_sensor(reg_conf_tbl->waddr, reg_conf_tbl->wdata);
			if (rc < 0)
			    return rc;
/*			rc = mt9m114_i2c_write(mt9m114_client->addr, reg_conf_tbl->waddr, reg_conf_tbl->wdata, reg_conf_tbl->width);  */
		}

		if (rc < 0)
			break;

		reg_conf_tbl++;
	}

	return rc;
}


static void mt9m114_start_stream(void)
{
	/* STANDBY_MODE_OFF   */
	mt9m114_i2c_write_w_sensor(0x098E, 0xDC00);
	mt9m114_i2c_write_w_sensor(0x0990, 0x5400);
	mt9m114_i2c_write_w_sensor(0x0080, 0x8002);
	mdelay(10);
}

static void mt9m114_stop_stream(void)
{
	/*  STANDBY_MODE_ON  */
	mt9m114_i2c_write_w_sensor(0x098E, 0xDC00);
	mt9m114_i2c_write_w_sensor(0x0990, 0x5000);
	mt9m114_i2c_write_w_sensor(0x0080, 0x8002);
	mdelay(10);
}

static int mt9m114_set_effect(int effect)
{
	int32_t rc = 0;

	pr_info("%s: effect = %d\n", __func__, effect);

	if (prev_effect_mode == effect) {
		pr_info("%s: skip this function, effect_mode -> %d\n", __func__, effect);
		return rc;
	}

	switch (effect) {
	case CAMERA_EFFECT_OFF:
		rc = mt9m114_i2c_write_table(mt9m114_regs.effect_default_tbl,
			mt9m114_regs.effect_default_tbl_size);
		break;
	case CAMERA_EFFECT_MONO:
		rc = mt9m114_i2c_write_table(mt9m114_regs.effect_mono_tbl,
			mt9m114_regs.effect_mono_tbl_size);
		break;
	case CAMERA_EFFECT_NEGATIVE:
		rc = mt9m114_i2c_write_table(mt9m114_regs.effect_negative_tbl,
			mt9m114_regs.effect_negative_tbl_size);
		break;
	case CAMERA_EFFECT_SOLARIZE:
		rc = mt9m114_i2c_write_table(mt9m114_regs.effect_solarization_tbl,
			mt9m114_regs.effect_solarization_tbl_size);
		break;
	case CAMERA_EFFECT_SEPIA:
		rc = mt9m114_i2c_write_table(mt9m114_regs.effect_sepia_tbl,
			mt9m114_regs.effect_sepia_tbl_size);
		break;
	case CAMERA_EFFECT_AQUA:
		rc = mt9m114_i2c_write_table(mt9m114_regs.effect_aqua_tbl,
			mt9m114_regs.effect_aqua_tbl_size);
		break;
	case CAMERA_EFFECT_POSTERIZE: /* effect off code */
		rc = mt9m114_i2c_write_table(mt9m114_regs.effect_default_tbl,
			mt9m114_regs.effect_default_tbl_size);
		break;
	default:
		return -EINVAL;
	}

	if (rc < 0)
		return rc;

	rc = mt9m114_i2c_write_w_sensor(0x0080, 0x8004);
	if (rc < 0)
		return rc;

	prev_effect_mode = effect;

	return rc;
}

static int mt9m114_set_wb(int mode)
{
	int32_t rc = 0;

	pr_info("%s: mode = %d\n", __func__, mode);

	if (prev_balance_mode == mode) {
		pr_info("%s: skip this function, wb_mode -> %d\n", __func__, mode);
		return rc;
	}

	switch (mode) {
	case CAMERA_WB_AUTO:
		rc = mt9m114_i2c_write_table(mt9m114_regs.wb_default_tbl,
			mt9m114_regs.wb_default_tbl_size);
		break;
	case CAMERA_WB_DAYLIGHT:	// sunny
		rc = mt9m114_i2c_write_table(mt9m114_regs.wb_sunny_tbl,
			mt9m114_regs.wb_sunny_tbl_size);
		break;
	case CAMERA_WB_CLOUDY_DAYLIGHT:  // cloudy
		rc = mt9m114_i2c_write_table(mt9m114_regs.wb_cloudy_tbl,
			mt9m114_regs.wb_cloudy_tbl_size);
		break;
	case CAMERA_WB_FLUORESCENT:
		rc = mt9m114_i2c_write_table(mt9m114_regs.wb_fluorescent_tbl,
			mt9m114_regs.wb_fluorescent_tbl_size);
		break;
	case CAMERA_WB_INCANDESCENT:
		rc = mt9m114_i2c_write_table(mt9m114_regs.wb_incandescent_tbl,
			mt9m114_regs.wb_incandescent_tbl_size);
		break;
	default:
		return -EINVAL;
	}

	return rc;
}

static int32_t mt9m114_sensor_setting(int update_type, int rt)
{
	int32_t rc = 0;

	pr_err("%s: update_type = %d, rt = %d\n", __func__, update_type, rt);

	switch (update_type) {
	case REG_INIT:

		/* stop streaming */
		mt9m114_stop_stream();
		msleep(15);

		/* PLL setup and initial setting */
		rc = mt9m114_i2c_write_table(mt9m114_regs.init_tbl,
			mt9m114_regs.inittbl_size);
		if (rc < 0) {
			pr_err("%s: mt9m114_i2c_write_table(init) failed!\n", __func__);
			return rc;
		}

		config_csi = 0;
		break;
	case UPDATE_PERIODIC:

		/* stop streaming */
		mt9m114_stop_stream();
		msleep(15);

		if (config_csi == 0) {
			struct msm_camera_csid_params mt9m114_csid_params;
			struct msm_camera_csiphy_params mt9m114_csiphy_params;
			struct msm_camera_csid_vc_cfg mt9m114_vccfg[] = {
				{0, 0x1E, CSI_DECODE_8BIT}, /*{0, CSI_RAW8, CSI_DECODE_8BIT},*/
				{1, CSI_EMBED_DATA, CSI_DECODE_8BIT},
			};
			mt9m114_csid_params.lane_cnt = 1;
			mt9m114_csid_params.lane_assign = 0xE4;
			mt9m114_csid_params.lut_params.num_cid =
				ARRAY_SIZE(mt9m114_vccfg);
			mt9m114_csid_params.lut_params.vc_cfg =
				&mt9m114_vccfg[0];
			mt9m114_csiphy_params.lane_cnt = 1;
			mt9m114_csiphy_params.settle_cnt = 0x14;
			rc = msm_camio_csid_config(&mt9m114_csid_params);
			v4l2_subdev_notify(mt9m114_ctrl->sensor_dev,
					NOTIFY_CID_CHANGE, NULL);
			dsb();
			rc = msm_camio_csiphy_config(&mt9m114_csiphy_params);
			dsb();

			msleep(10);
			config_csi = 1;
		}

/*
		if (rt == QTR_SIZE) {
			mt9m114_i2c_write_table(mt9m114_regs.preview_size_full_tbl,
				mt9m114_regs.preview_size_full_tbl_size);*/
/*			mt9m114_i2c_write_table(mt9m114_regs.preview_size_vga_tbl,
				mt9m114_regs.preview_size_vga_tbl_size); */
/*		} else if (rt == FULL_SIZE) {
			mt9m114_i2c_write_table(mt9m114_regs.preview_size_full_tbl,
				mt9m114_regs.preview_size_full_tbl_size);
		}
*/

		mt9m114_start_stream();

#if 0 /* For Test */
		/* Start polling CSIPHY interrupt status register here */
		do {
			msm_io_read_interrupt();
		} while (Cnt++ < 20);
#endif

		break;
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int32_t mt9m114_video_config(int mode)
{
	int32_t rc = 0;

	pr_info("%s: start\n", __func__);

	rc = mt9m114_i2c_write_table(mt9m114_regs.prev_tbl, mt9m114_regs.prevtbl_size);
	if (rc < 0) {
		pr_err("%s: mt9m114_i2c_write_table(prev) failed!\n", __func__);
		return rc;
	}

	/* Change sensor resolution if needed */
	rc = mt9m114_sensor_setting(UPDATE_PERIODIC, mt9m114_ctrl->prev_res);
	if (rc < 0)
		return rc;

	mt9m114_ctrl->curr_res = mt9m114_ctrl->prev_res;
	mt9m114_ctrl->sensormode = mode;

	return rc;
}

static int32_t mt9m114_snapshot_config(int mode)
{
	int32_t rc = 0;

	pr_info("%s: start\n", __func__);

	/* Change sensor resolution if needed */
	if (mt9m114_ctrl->curr_res != mt9m114_ctrl->pict_res) {
		if (mt9m114_sensor_setting(UPDATE_PERIODIC,
				mt9m114_ctrl->pict_res) < 0)
			return rc;
	}

	mt9m114_ctrl->curr_res = mt9m114_ctrl->pict_res;
	mt9m114_ctrl->sensormode = mode;
	return rc;
}

static int32_t mt9m114_raw_snapshot_config(int mode)
{
	int32_t rc = 0;

	pr_info("%s: start\n", __func__);

	/* Change sensor resolution if needed */
	if (mt9m114_ctrl->curr_res != mt9m114_ctrl->pict_res) {
		if (mt9m114_sensor_setting(UPDATE_PERIODIC,
				mt9m114_ctrl->pict_res) < 0)
			return rc;
	}

	mt9m114_ctrl->curr_res = mt9m114_ctrl->pict_res;
	mt9m114_ctrl->sensormode = mode;
	return rc;
}

static int32_t mt9m114_set_sensor_mode(int mode,	int res)
{
	int32_t rc = 0;

	pr_err("%s: mode = %d, res = %d\n", __func__, mode, res);

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		mt9m114_ctrl->pict_res = res;
		rc = mt9m114_video_config(mode);
		break;
	case SENSOR_SNAPSHOT_MODE:
		mt9m114_ctrl->pict_res = res;
		rc = mt9m114_snapshot_config(mode);
		break;
	case SENSOR_RAW_SNAPSHOT_MODE:
		mt9m114_ctrl->pict_res = res;
		rc = mt9m114_raw_snapshot_config(mode);
		break;
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int32_t mt9m114_power_down(void)
{
	return 0;
}

static int mt9m114_probe_init_done(const struct msm_camera_sensor_info *data)
{
	gpio_direction_output(data->sensor_reset, 0);
/*	gpio_set_value_cansleep(data->sensor_reset, 0); */
	gpio_free(data->sensor_reset);
	return 0;
}

static int mt9m114_probe_init_sensor(const struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;
	uint16_t chipid = 0;

	pr_info("%s: Entered!\n", __func__);

	/* Reset */
	rc = gpio_request(data->sensor_reset, "mt9m114");
	if (!rc) {
		rc = gpio_direction_output(data->sensor_reset, 0);
		CDBG("%s: sensor_reset = %d, rc = %d\n", __func__, data->sensor_reset, rc);
		mdelay(60);
		rc = gpio_direction_output(data->sensor_reset, 1);
		CDBG("%s: sensor_reset = %d, rc = %d\n", __func__, data->sensor_reset, rc);
		mdelay(50);
	} else {
		goto probe_init_sensor_fail;
	}

	/* Compare sensor ID to MT9M114 ID: 0x2481 */
	rc = mt9m114_i2c_read(0x0000, &chipid, 2);
	pr_err("Sensor ID: %d(0x%x)\n", chipid, chipid);
	if (chipid != 0x2481) {
		rc = -ENODEV;
		pr_err("mt9m114_probe_init_sensor fail chip id doesnot match\n");
		goto probe_init_sensor_fail;
	}

	pr_err(" mt9m114_probe_init_sensor finishes\n");
	return rc;

probe_init_sensor_fail:
	pr_err(" mt9m114_probe_init_sensor fails\n");
	mt9m114_probe_init_done(data);
	return rc;
}

static int mt9m114_sensor_open_init(const struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;

	pr_info("%s: Entered!\n", __func__);

	mt9m114_ctrl->fps_divider = 1 * 0x00000400;
	mt9m114_ctrl->pict_fps_divider = 1 * 0x00000400;
	mt9m114_ctrl->fps = 30*Q8;
	mt9m114_ctrl->set_test = TEST_OFF;
	mt9m114_ctrl->prev_res = QTR_SIZE;
	mt9m114_ctrl->pict_res = FULL_SIZE;

	if (data)
		mt9m114_ctrl->sensordata = data;

	/* Set clock */
	msm_camio_clk_rate_set(MT9M114_MASTER_CLK_RATE);

	/* Sensor reset & ID check */
	rc = mt9m114_probe_init_sensor(data);
	if (rc < 0)
		goto sensor_open_init_fail;

	/* PLL setup and initial setting */
	rc = mt9m114_sensor_setting(REG_INIT, mt9m114_ctrl->prev_res);
	if (rc < 0)
		goto sensor_open_init_fail;

	pr_err("mt9m114_sensor_open_init done\n");
	return rc;

sensor_open_init_fail:
	pr_err("mt9m114_sensor_open_init fail\n");
	mt9m114_probe_init_done(data);
	return rc;
}

static int mt9m114_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&mt9m114_wait_queue);
	return 0;
}

static int mt9m114_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	pr_err("mt9m114_i2c_probe called!\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		rc = -ENOTSUPP;
		pr_err("i2c_check_functionality failed\n");
		goto i2c_probe_failure;
	}

	mt9m114_sensorw = kzalloc(sizeof(struct mt9m114_work), GFP_KERNEL);
	if (!mt9m114_sensorw) {
		rc = -ENOMEM;
		pr_err("kzalloc failed.\n");
		goto i2c_probe_failure;
	}

	i2c_set_clientdata(client, mt9m114_sensorw);
	mt9m114_init_client(client);
	mt9m114_client = client;

	pr_err("mt9m114_i2c_probe successed! rc = %d\n", rc);
	return 0;

i2c_probe_failure:
	if (mt9m114_sensorw != NULL)
		kfree(mt9m114_sensorw);
	mt9m114_sensorw = NULL;
	pr_err("mt9m114_i2c_probe failed! rc = %d\n", rc);
	return rc;
}

static int __exit mt9m114_remove(struct i2c_client *client)
{
	struct mt9m114_work *sensorw = i2c_get_clientdata(client);
	free_irq(client->irq, sensorw);
	mt9m114_client = NULL;
	kfree(sensorw);
	return 0;
}

static const struct i2c_device_id mt9m114_i2c_id[] = {
	{"mt9m114", 0},
	{ }
};

static struct i2c_driver mt9m114_i2c_driver = {
	.id_table = mt9m114_i2c_id,
	.probe    = mt9m114_i2c_probe,
	.remove   = __exit_p(mt9m114_i2c_remove),
	.driver   = {
		.name = "mt9m114",
	},
};

static int mt9m114_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cdata;
	long   rc = 0;

	if (copy_from_user(&cdata,
			(void *)argp,
			sizeof(struct sensor_cfg_data)))
		return -EFAULT;

	pr_info("%s: cfgtype = %d, mode = %d\n",
		__func__, cdata.cfgtype, cdata.mode);

	mutex_lock(&mt9m114_mut);

	switch (cdata.cfgtype) {
	case CFG_GET_PICT_FPS:
	case CFG_GET_PREV_L_PF:
	case CFG_GET_PREV_P_PL:
	case CFG_GET_PICT_L_PF:
	case CFG_GET_PICT_P_PL:
	case CFG_GET_PICT_MAX_EXP_LC:
	case CFG_SET_FPS:
	case CFG_SET_PICT_FPS:
	case CFG_SET_EXP_GAIN:
	case CFG_SET_PICT_EXP_GAIN:
		rc = -EFAULT;
		break;

	case CFG_SET_MODE:
		rc = mt9m114_set_sensor_mode(cdata.mode, cdata.rs);
		break;
	case CFG_SET_EFFECT:
		rc = mt9m114_set_effect(cdata.mode);
		break;
	case CFG_SET_WB:
		rc = mt9m114_set_wb(cdata.mode);
		break;
	default:
		rc = -EFAULT;
		break;
	}

	mutex_unlock(&mt9m114_mut);

	return rc;
}

static int mt9m114_sensor_release(void)
{
	int rc = -EBADF;

	pr_info("%s: start\n", __func__);

	mutex_lock(&mt9m114_mut);
	mt9m114_power_down();
	gpio_direction_output(mt9m114_ctrl->sensordata->sensor_reset, 0);
/*
	gpio_set_value_cansleep(mt9m114_ctrl->sensordata->sensor_reset, 0);
	msleep(5);
*/
	gpio_free(mt9m114_ctrl->sensordata->sensor_reset);
	mutex_unlock(&mt9m114_mut);
	pr_err("%s: mt9m114_release completed\n", __func__);

	return rc;
}

static int mt9m114_sensor_probe(const struct msm_camera_sensor_info *info,
				struct msm_sensor_ctrl *s)
{
	int rc = 0;

	pr_info("%s: start\n", __func__);

	rc = i2c_add_driver(&mt9m114_i2c_driver);
	if (rc < 0 || mt9m114_client == NULL) {
		rc = -ENOTSUPP;
		pr_err("I2C add driver failed");
		goto probe_fail;
	}

	/* Set clock */
	msm_camio_clk_rate_set(MT9M114_MASTER_CLK_RATE);

	/* Reset sensor and probe sensor registers thru I2C */
	rc = mt9m114_probe_init_sensor(info);
	if (rc < 0)
		goto probe_fail;

	s->s_init = mt9m114_sensor_open_init;
	s->s_release = mt9m114_sensor_release;
	s->s_config = mt9m114_sensor_config;
	s->s_camera_type = FRONT_CAMERA_2D;
	s->s_mount_angle = info->sensor_platform_info->mount_angle;

	mt9m114_probe_init_done(info);

	pr_info("mt9m114_sensor_probe : SUCCESS!\n");
	return rc;

probe_fail:
	pr_err("mt9m114_sensor_probe: SENSOR PROBE FAILS!\n");
	return rc;
}

static struct mt9m114_format mt9m114_subdev_info[] = {
	{
		.code       = V4L2_MBUS_FMT_YUYV8_2X8, /* For YUV type sensor (YUV422) */
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt        = 1,
		.order      = 0,
	},
	/* more can be supported, to be added later */
};

static int mt9m114_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
			   enum v4l2_mbus_pixelcode *code)
{
	printk(KERN_DEBUG "Index is %d\n", index);
	if ((unsigned int)index >= ARRAY_SIZE(mt9m114_subdev_info))
		return -EINVAL;

	*code = mt9m114_subdev_info[index].code;
	return 0;
}

static struct v4l2_subdev_core_ops mt9m114_subdev_core_ops;
static struct v4l2_subdev_video_ops mt9m114_subdev_video_ops = {
	.enum_mbus_fmt = mt9m114_enum_fmt,
};

static struct v4l2_subdev_ops mt9m114_subdev_ops = {
	.core   = &mt9m114_subdev_core_ops,
	.video  = &mt9m114_subdev_video_ops,
};

static int mt9m114_sensor_probe_cb(const struct msm_camera_sensor_info *info,
	struct v4l2_subdev *sdev, struct msm_sensor_ctrl *s)
{
	int rc = 0;

	mt9m114_ctrl = kzalloc(sizeof(struct mt9m114_ctrl_t), GFP_KERNEL);
	if (!mt9m114_ctrl) {
		pr_err("%s: kzalloc failed!\n", __func__);
		return -ENOMEM;
	}

	rc = mt9m114_sensor_probe(info, s);
	if (rc < 0) {
		pr_err("%s: mt9m114_sensor_probe fail\n", __func__);
		return rc;
	}

	/* probe is successful, init a v4l2 subdevice */
	printk(KERN_DEBUG "going into v4l2_i2c_subdev_init\n");
	if (sdev) {
		v4l2_i2c_subdev_init(sdev, mt9m114_client,
						&mt9m114_subdev_ops);
		mt9m114_ctrl->sensor_dev = sdev;
	}
	return rc;
}

static int __mt9m114_probe(struct platform_device *pdev)
{
	pr_err("####### __mt9m114_probe()");
	return msm_sensor_register(pdev, mt9m114_sensor_probe_cb);
}

static struct platform_driver msm_camera_driver = {
	.probe = __mt9m114_probe,
	.driver = {
		.name = "msm_camera_mt9m114",
		.owner = THIS_MODULE,
	},
};

static int __init mt9m114_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

module_init(mt9m114_init);
MODULE_DESCRIPTION("Aptina 1.26 MP SoC sensor driver");
MODULE_LICENSE("GPL v2");

