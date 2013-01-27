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

#include "msm_actuator.h"
#include "msm_camera_i2c.h"

#define REG_VCM_CONTROL			0x30F0
#define REG_VCM_NEW_CODE			0x30F2
#define REG_VCM_STEP_TIME			0x30F4

#define	MT9E013_TOTAL_STEPS_NEAR_TO_FAR			37

DEFINE_MUTEX(mt9e013_act_mutex);
static struct msm_actuator_ctrl_t mt9e013act_t;

static struct region_params_t g_regions[] = {
	/* step_bound[0] - macro side boundary
	* step_bound[1] - infinity side boundary
	*/
	/* Region 1 */
	{
		.step_bound = {2, 0},
		.code_per_step = 25,
	},
	/* Region 2 */
	{
		.step_bound = {MT9E013_TOTAL_STEPS_NEAR_TO_FAR, 2},
		.code_per_step = 3,
	}
};

int32_t mt9e013_camera_i2c_txdata(struct msm_camera_i2c_client *dev_client,
		unsigned char *txdata, int length)
{
	int32_t rc = 0;
	uint16_t saddr = dev_client->client->addr;/* addr >> 1; */
	struct i2c_msg msg[] = {
		{
			.addr = saddr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		},
	};
	rc = i2c_transfer(dev_client->client->adapter, msg, 1);
	if (rc < 0)
		pr_err("mt9e013_camera_i2c_txdata faild 0x%x\n", saddr);
	return 0;
}

static int32_t mt9e013_camera_i2c_write(struct msm_camera_i2c_client *client,
		uint16_t addr, uint16_t data,
		enum msm_camera_i2c_data_type data_type)
{
	int32_t rc = -EFAULT;
	unsigned char buf[client->addr_type + data_type];

	buf[0] = addr >> BITS_PER_BYTE;
	buf[1] = addr;
	buf[2] = data >> BITS_PER_BYTE;
	buf[3] = data;

	rc = mt9e013_camera_i2c_txdata(client, buf, 4);
	if (rc < 0)
		pr_err("i2c_write_b failed, addr = 0x%x, val = 0x%x!\n",
				addr, data);
	return rc;
}


static int32_t mt9e013_wrapper_i2c_write(struct msm_actuator_ctrl_t *a_ctrl,
		int16_t next_lens_position, void *params)
{

	mt9e013_camera_i2c_write(&a_ctrl->i2c_client,
			REG_VCM_NEW_CODE, next_lens_position, MSM_CAMERA_I2C_WORD_DATA);
	return 0;
}

static int32_t mt9e013_act_init_focus(struct msm_actuator_ctrl_t *a_ctrl)
{
	return mt9e013_camera_i2c_write(&a_ctrl->i2c_client,
			REG_VCM_NEW_CODE, 0x00, MSM_CAMERA_I2C_WORD_DATA);
}

static int32_t mt9e013_act_move_focus(struct msm_actuator_ctrl_t *a_ctrl,
		int dir, int32_t num_steps)
{
	int32_t rc = 0;
	int16_t sign_dir, dest_lens_position, dest_step_pos;
	uint16_t vcm_step_time;
	uint16_t wait_time;
	uint16_t curr_scene = 0;
	uint16_t scenario_size = 0;
	uint16_t index = 0;
	uint16_t step_boundary = 0;

	LINFO("%s called, dir %d, num_steps %d\n",
			__func__, dir, num_steps);

	/* Determine sign direction */
	if (dir == MOVE_NEAR)
		sign_dir = 1;
	else if (dir == MOVE_FAR)
		sign_dir = -1;
	else {
		pr_err("Illegal focus direction\n");
		rc = -EINVAL;
		return rc;
	}

	/* Determine destination step position */
	dest_step_pos = a_ctrl->curr_step_pos +
			(sign_dir * num_steps);

	if (dest_step_pos < 0)
		dest_step_pos = 0;
	else if (dest_step_pos > a_ctrl->set_info.total_steps)
		dest_step_pos = a_ctrl->set_info.total_steps;

	if (dest_step_pos == a_ctrl->curr_step_pos)
		return rc;

	/* Determine scenario */
	scenario_size = a_ctrl->scenario_size[dir];
	for (index = 0; index < scenario_size; index++) {
		if (num_steps <= a_ctrl->ringing_scenario[dir][index]) {
			curr_scene = index;
			break;
		}
	}

	dest_lens_position = a_ctrl->step_position_table[dest_step_pos];

	if ((dest_step_pos <= 4) && (sign_dir == 1)) {
		step_boundary =
			a_ctrl->region_params[a_ctrl->curr_region_index].step_bound[dir];

		mt9e013_camera_i2c_write(&a_ctrl->i2c_client,
				REG_VCM_STEP_TIME, 0x0000, MSM_CAMERA_I2C_WORD_DATA);

		if (num_steps == 4) {
			LINFO("__debug:MoveFocus, jumpvalue:%d \n",
					step_boundary * a_ctrl->region_params
					[a_ctrl->curr_region_index].code_per_step);

			rc = a_ctrl->func_tbl.actuator_i2c_write(
					a_ctrl,
					step_boundary * a_ctrl->region_params
					[a_ctrl->curr_region_index].code_per_step, NULL);
		} else {
			if (dest_step_pos <= step_boundary) {
				LINFO("__debug:MoveFocus, fine search:%d \n",
						dest_lens_position);

				rc = a_ctrl->func_tbl.actuator_i2c_write(
						a_ctrl,
						dest_lens_position, NULL);

				a_ctrl->curr_step_pos = dest_step_pos;
				return 0;
			}
		}
	}

	if (sign_dir < 0) {
		if (num_steps > 20) {
			/*macro to infinity*/
			vcm_step_time = 0x0050;
			wait_time = 5;
		} else if (num_steps <= 4) {
			/*reverse search fine step  dir - macro to infinity*/
			vcm_step_time = 0x0400;
			wait_time = 4;
		} else {
			/*reverse search Coarse Jump ( > 4) dir - macro to infinity*/
			vcm_step_time = 0x96;
			wait_time = 3;
		}
	} else {
		if (num_steps >= 4) {
			/*coarse jump  dir - infinity to macro*/
			vcm_step_time = 0x0200;
			wait_time = 2;
		} else {
			/*fine step  dir - infinity to macro*/
			vcm_step_time = 0x0400;
			wait_time = 4;
		}
	}

	LINFO("MoveFocus, vcm_step_time:%d dest_lens_position:%d  \n",
			vcm_step_time, dest_lens_position);

	mt9e013_camera_i2c_write(&a_ctrl->i2c_client,
			REG_VCM_STEP_TIME, vcm_step_time, MSM_CAMERA_I2C_WORD_DATA);

	if (a_ctrl->curr_step_pos != dest_step_pos) {
		mt9e013_camera_i2c_write(&a_ctrl->i2c_client,
				REG_VCM_NEW_CODE, dest_lens_position, MSM_CAMERA_I2C_WORD_DATA);

		usleep(wait_time * 1000);
	}
	a_ctrl->curr_step_pos = dest_step_pos;
	return 0;
}

static const struct i2c_device_id mt9e013act_i2c_id[] = {
	{"mt9e013_act", (kernel_ulong_t)&mt9e013act_t},
	{ }
};

static int mt9e013_act_config(
		void __user *argp)
{
	LINFO("%s called\n", __func__);
	return (int) msm_actuator_config(&mt9e013act_t, argp);
}

static int mt9e013_i2c_add_driver_table(
		void)
{
	LINFO("%s called\n", __func__);
	return (int) msm_actuator_init_table(&mt9e013act_t);
}

static struct i2c_driver mt9e013act_i2c_driver = {
	.id_table = mt9e013act_i2c_id,
	.probe  = msm_actuator_i2c_probe,
	.remove = __exit_p(mt9e013act_i2c_remove),
	.driver = {
		.name = "mt9e013_act",
	},
};

static int __init mt9e013_i2c_add_driver(
		void)
{
	LINFO("%s called\n", __func__);

	return i2c_add_driver(mt9e013act_t.i2c_driver);
}

static struct v4l2_subdev_core_ops mt9e013act_subdev_core_ops;

static struct v4l2_subdev_ops mt9e013act_subdev_ops = {
	.core = &mt9e013act_subdev_core_ops,
};

static int32_t mt9e013_act_create_subdevice(
		void *board_info,
		void *sdev)
{
	LINFO("%s called\n", __func__);

	return (int) msm_actuator_create_subdevice(&mt9e013act_t,
			(struct i2c_board_info const *)board_info,
			(struct v4l2_subdev *)sdev);
}

static struct msm_actuator_ctrl_t mt9e013act_t = {
	.i2c_driver = &mt9e013act_i2c_driver,
	.i2c_addr = 0x6C >> 1,
	.act_v4l2_subdev_ops = &mt9e013act_subdev_ops,
	.actuator_ext_ctrl = {
		.a_init_table = mt9e013_i2c_add_driver_table,
		.a_create_subdevice = mt9e013_act_create_subdevice,
		.a_config = mt9e013_act_config,
	},

	.i2c_client = {
		.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
	},

	.set_info = {
		.total_steps = MT9E013_TOTAL_STEPS_NEAR_TO_FAR,
	},

	.curr_step_pos = 0,
	.curr_region_index = 0,
	.initial_code = 0,		/* init value of  step_position_table[0]*/
	.actuator_mutex = &mt9e013_act_mutex,

	/* Initialize region params */
	.region_params = g_regions,
	.region_size = ARRAY_SIZE(g_regions),

	.func_tbl = {
		.actuator_init_table = msm_actuator_init_table,
		.actuator_init_focus = mt9e013_act_init_focus,
		.actuator_move_focus = mt9e013_act_move_focus,
		.actuator_set_default_focus = msm_actuator_set_default_focus,
		.actuator_i2c_write = mt9e013_wrapper_i2c_write,
	},
};

subsys_initcall(mt9e013_i2c_add_driver);
MODULE_DESCRIPTION("MT9E013 actuator");
MODULE_LICENSE("GPL v2");

