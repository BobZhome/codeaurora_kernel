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
#include <linux/debugfs.h>

#define S5K4E1_TOTAL_STEPS_NEAR_TO_FAR_MAX 44

DEFINE_MUTEX(s5k4e1_act_mutex);
static int s5k4e1_actuator_debug_init(void);
static struct msm_actuator_ctrl_t s5k4e1_act_t;

static int32_t s5k4e1_wrapper_i2c_write(struct msm_actuator_ctrl_t *a_ctrl,
	int16_t next_lens_position, void *params)
{
	uint16_t msb = 0, lsb = 0;
	msb = (next_lens_position >> 4) & 0x3F;
	lsb = (next_lens_position << 4) & 0xF0;
	lsb |= (*(uint8_t *)params);
	CDBG("%s: Actuator MSB:0x%x, LSB:0x%x\n", __func__, msb, lsb);
	msm_camera_i2c_write(&a_ctrl->i2c_client,
		msb, lsb, MSM_CAMERA_I2C_BYTE_DATA);
	return next_lens_position;
}

static uint8_t s5k4e1_hw_params[] = {
	0x0,
	0x5,
	0x6,
	0x8,
	0xB,
};

static uint16_t s5k4e1_macro_scenario[] = {
	/* MOVE_NEAR dir*/
	4,
	29,
};

static uint16_t s5k4e1_inf_scenario[] = {
	/* MOVE_FAR dir */
	8,
	22,
	29,
};

static struct region_params_t s5k4e1_regions[] = {
	/* step_bound[0] - macro side boundary
	 * step_bound[1] - infinity side boundary
	 */
	/* Region 1 */
	{
		.step_bound = {2, 0},
		.code_per_step = 80,
	},
	/* Region 2 */
	{
		.step_bound = {44, 2},
		.code_per_step = 10,
	}
};

static struct damping_params_t s5k4e1_macro_reg1_damping[] = {
	/* MOVE_NEAR Dir */
	/* Scene 1 => Damping params */
	{
		.damping_step = 0xFF,
		.damping_delay = 1500,
		.hw_params = &s5k4e1_hw_params[0],
	},
	/* Scene 2 => Damping params */
	{
		.damping_step = 0xFF,
		.damping_delay = 1500,
		.hw_params = &s5k4e1_hw_params[0],
	},
};

static struct damping_params_t s5k4e1_macro_reg2_damping[] = {
	/* MOVE_NEAR Dir */
	/* Scene 1 => Damping params */
	{
		.damping_step = 0xFF,
		.damping_delay = 4500,
		.hw_params = &s5k4e1_hw_params[4],
	},
	/* Scene 2 => Damping params */
	{
		.damping_step = 0xFF,
		.damping_delay = 4500,
		.hw_params = &s5k4e1_hw_params[3],
	},
};

static struct damping_params_t s5k4e1_inf_reg1_damping[] = {
	/* MOVE_FAR Dir */
	/* Scene 1 => Damping params */
	{
		.damping_step = 0xFF,
		.damping_delay = 450,
		.hw_params = &s5k4e1_hw_params[0],
	},
	/* Scene 2 => Damping params */
	{
		.damping_step = 0xFF,
		.damping_delay = 450,
		.hw_params = &s5k4e1_hw_params[0],
	},
	/* Scene 3 => Damping params */
	{
		.damping_step = 0xFF,
		.damping_delay = 450,
		.hw_params = &s5k4e1_hw_params[0],
	},
};

static struct damping_params_t s5k4e1_inf_reg2_damping[] = {
	/* MOVE_FAR Dir */
	/* Scene 1 => Damping params */
	{
		.damping_step = 0x1FF,
		.damping_delay = 4500,
		.hw_params = &s5k4e1_hw_params[2],
	},
	/* Scene 2 => Damping params */
	{
		.damping_step = 0x1FF,
		.damping_delay = 4500,
		.hw_params = &s5k4e1_hw_params[1],
	},
	/* Scene 3 => Damping params */
	{
		.damping_step = 27,
		.damping_delay = 2700,
		.hw_params = &s5k4e1_hw_params[0],
	},
};

static struct damping_t s5k4e1_macro_regions[] = {
	/* MOVE_NEAR dir */
	/* Region 1 */
	{
		.ringing_params = s5k4e1_macro_reg1_damping,
	},
	/* Region 2 */
	{
		.ringing_params = s5k4e1_macro_reg2_damping,
	},
};

static struct damping_t s5k4e1_inf_regions[] = {
	/* MOVE_FAR dir */
	/* Region 1 */
	{
		.ringing_params = s5k4e1_inf_reg1_damping,
	},
	/* Region 2 */
	{
		.ringing_params = s5k4e1_inf_reg2_damping,
	},
};


static int32_t s5k4e1_set_params(struct msm_actuator_ctrl_t *a_ctrl)
{
	return 0;
}

static const struct i2c_device_id s5k4e1_act_i2c_id[] = {
	{"s5k4e1_act", (kernel_ulong_t)&s5k4e1_act_t},
	{ }
};

static int s5k4e1_act_config(
	void __user *argp)
{
	LINFO("%s called\n", __func__);
	return (int) msm_actuator_config(&s5k4e1_act_t, argp);
}

static int s5k4e1_i2c_add_driver_table(
	void)
{
	LINFO("%s called\n", __func__);
	return (int) msm_actuator_init_table(&s5k4e1_act_t);
}

static struct i2c_driver s5k4e1_act_i2c_driver = {
	.id_table = s5k4e1_act_i2c_id,
	.probe  = msm_actuator_i2c_probe,
	.remove = __exit_p(s5k4e1_act_i2c_remove),
	.driver = {
		.name = "s5k4e1_act",
	},
};

static int __init s5k4e1_i2c_add_driver(
	void)
{
	LINFO("%s called\n", __func__);
	return i2c_add_driver(s5k4e1_act_t.i2c_driver);
}

static struct v4l2_subdev_core_ops s5k4e1_act_subdev_core_ops;

static struct v4l2_subdev_ops s5k4e1_act_subdev_ops = {
	.core = &s5k4e1_act_subdev_core_ops,
};

static int32_t s5k4e1_act_probe(
	void *board_info,
	void *sdev)
{
	LINFO("%s called\n", __func__);
	s5k4e1_actuator_debug_init();

	return (int) msm_actuator_create_subdevice(&s5k4e1_act_t,
		(struct i2c_board_info const *)board_info,
		(struct v4l2_subdev *)sdev);
}

/* LGE_CHANGE_S, Actuator noise fix when exit, 2012.04.12 yousung.kang@lge.com */
int32_t s5k4e1_act_power_down(void *a_info)
{
	int32_t rc = 0;
	rc = s5k4e1_act_t.func_tbl.actuator_power_down(&s5k4e1_act_t);
	return rc;
}
/* LGE_CHANGE_E, Actuator noise fix when exit, 012.04.12 yousung.kang@lge.com */

/* LGE_CHANGE_S, Noise fix using h/w damping, 2012.04.16 yousung.kang@lge.com */
int32_t s5k4e1_actuator_set_default_focus(
	struct msm_actuator_ctrl_t *a_ctrl)
{
	int32_t rc = 0;
	if (!a_ctrl->step_position_table)
		a_ctrl->func_tbl.actuator_init_table(a_ctrl);

	if (a_ctrl->curr_step_pos != 0) {
		uint8_t hw_damping = 0xF;  // use DW9716

		s5k4e1_wrapper_i2c_write(a_ctrl, a_ctrl->initial_code, &hw_damping);
		mdelay(50); 	// delay can be changed but put max due to it's small enough

		a_ctrl->curr_step_pos = 0;
	}

	return rc;
}

int32_t s5k4e1_actuator_move_focus(
	struct msm_actuator_ctrl_t *a_ctrl,
	int dir,
	int32_t num_steps)
{
	int8_t sign_dir = 0;
	int16_t dest_step_pos = 0;

	/* Determine sign direction */
	if (dir == MOVE_NEAR)
		sign_dir = 1;
	else if (dir == MOVE_FAR)
		sign_dir = -1;
	else {
		pr_err("Illegal focus direction\n");
		return -EINVAL;
	}

	/* Determine destination step position */
	dest_step_pos = a_ctrl->curr_step_pos +
		(sign_dir * num_steps);

	if(dest_step_pos == 0)
		return s5k4e1_actuator_set_default_focus(a_ctrl);
	else
		return msm_actuator_move_focus(a_ctrl, dir, num_steps);
}
/* LGE_CHANGE_E, Noise fix using h/w damping, 2012.04.16 yousung.kang@lge.com */

int32_t s5k4e1_actuator_af_power_down(struct msm_actuator_ctrl_t *a_ctrl)
{
	int32_t rc = 0;
	LINFO("%s called\n", __func__);

	pr_info("%s: ",  __func__);


	if (a_ctrl->step_position_table[a_ctrl->curr_step_pos] !=
		a_ctrl->initial_code) {

/* LGE_CHANGE_S, Fix actuator noise when Camera close, 2012.04.13, yousung.kang@lge.com */
/*
		if(a_ctrl->curr_step_pos > 11)
		{
		  rc = a_ctrl->func_tbl.actuator_move_focus(a_ctrl, MOVE_FAR,
					(a_ctrl->curr_step_pos-11));
		  mdelay(100);

		}

		if(a_ctrl->curr_step_pos > 4)
		{
		  rc = a_ctrl->func_tbl.actuator_move_focus(a_ctrl, MOVE_FAR,
					(a_ctrl->curr_step_pos-4));
		  mdelay(100);

		}

		if(a_ctrl->curr_step_pos > 1)
		{
		  rc = a_ctrl->func_tbl.actuator_move_focus(a_ctrl, MOVE_FAR,
					(a_ctrl->curr_step_pos-1));
		  mdelay(100);

		}*/
/* LGE_CHANGE_E, Fix actuator noise when Camera close, 2012.04.13, yousung.kang@lge.com */

		rc = a_ctrl->func_tbl.actuator_set_default_focus(a_ctrl);
	    mdelay(100);

		LINFO("%s after msm_actuator_set_default_focus\n", __func__);
	}
	kfree(a_ctrl->step_position_table);
	return rc;
}



static struct msm_actuator_ctrl_t s5k4e1_act_t = {
	.i2c_driver = &s5k4e1_act_i2c_driver,
	.i2c_addr = 0x18,
	.act_v4l2_subdev_ops = &s5k4e1_act_subdev_ops,
	.actuator_ext_ctrl = {
		.a_init_table = s5k4e1_i2c_add_driver_table,
		.a_create_subdevice = s5k4e1_act_probe,
		.a_config = s5k4e1_act_config,
		/* LGE_CHANGE_S, Actuator noise fix when exit, 2012.04.12 yousung.kang@lge.com */
		.a_power_down = s5k4e1_act_power_down,
		/* LGE_CHANGE_E, Actuator noise fix when exit, 012.04.12 yousung.kang@lge.com */

	},

	.i2c_client = {
		.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	},

	.set_info = {
		.total_steps = S5K4E1_TOTAL_STEPS_NEAR_TO_FAR_MAX,
	},

	.curr_step_pos = 0,
	.curr_region_index = 0,
	.initial_code = 100,
	.actuator_mutex = &s5k4e1_act_mutex,

	/* Initialize scenario */
	.ringing_scenario[MOVE_NEAR] = s5k4e1_macro_scenario,
	.scenario_size[MOVE_NEAR] = ARRAY_SIZE(s5k4e1_macro_scenario),
	.ringing_scenario[MOVE_FAR] = s5k4e1_inf_scenario,
	.scenario_size[MOVE_FAR] = ARRAY_SIZE(s5k4e1_inf_scenario),

	/* Initialize region params */
	.region_params = s5k4e1_regions,
	.region_size = ARRAY_SIZE(s5k4e1_regions),

	/* Initialize damping params */
	.damping[MOVE_NEAR] = s5k4e1_macro_regions,
	.damping[MOVE_FAR] = s5k4e1_inf_regions,

	.func_tbl = {
		.actuator_set_params = s5k4e1_set_params,
		.actuator_init_focus = NULL,
		.actuator_init_table = msm_actuator_init_table,
		.actuator_move_focus = s5k4e1_actuator_move_focus, 	/* LGE_CHANGE, Noise fix using h/w damping, 2012.04.16 ku.kwon@lge.com */
		.actuator_write_focus = msm_actuator_write_focus,
		.actuator_i2c_write = s5k4e1_wrapper_i2c_write,
		.actuator_set_default_focus = s5k4e1_actuator_set_default_focus,	/* LGE_CHANGE, Noise fix using h/w damping, 2012.04.16 ku.kwon@lge.com */
/* LGE_CHANGE_S, Actuator noise fix when exit, 2012.04.12 yousung.kang@lge.com */
		.actuator_power_down = msm_actuator_af_power_down,
//		.actuator_power_down = s5k4e1_actuator_af_power_down,
/* LGE_CHANGE_E, Actuator noise fix when exit, 012.04.12 yousung.kang@lge.com */
	},

/* LGE_CHANGE S, CTS Fix for testParameters, 2012-03-01 ku.kwon@lge.com */
	.get_info = {
		.focal_length_num = 320,
		.focal_length_den = 100,
		.f_number_num = 240,
		.f_number_den = 100,
		.f_pix_num = 112,
		.f_pix_den = 100,
		.total_f_dist_num = 171016,
		.total_f_dist_den = 100,
		.hor_view_angle_num = 598,
		.hor_view_angle_den = 10,
		.ver_view_angle_num = 467,
		.ver_view_angle_den = 10,
	},
/* LGE_CHANGE E, CTS Fix for testParameters, 2012-03-01 ku.kwon@lge.com */

};

static int s5k4e1_actuator_set_delay(void *data, u64 val)
{
	s5k4e1_inf_reg2_damping[1].damping_delay = val;
	return 0;
}

static int s5k4e1_actuator_get_delay(void *data, u64 *val)
{
	*val = s5k4e1_inf_reg2_damping[1].damping_delay;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(s5k4e1_delay,
	s5k4e1_actuator_get_delay,
	s5k4e1_actuator_set_delay,
	"%llu\n");

static int s5k4e1_actuator_set_jumpparam(void *data, u64 val)
{
	s5k4e1_inf_reg2_damping[1].damping_step = val & 0xFFF;
	return 0;
}

static int s5k4e1_actuator_get_jumpparam(void *data, u64 *val)
{
	*val = s5k4e1_inf_reg2_damping[1].damping_step;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(s5k4e1_jumpparam,
	s5k4e1_actuator_get_jumpparam,
	s5k4e1_actuator_set_jumpparam,
	"%llu\n");

static int s5k4e1_actuator_set_hwparam(void *data, u64 val)
{
	s5k4e1_hw_params[2] = val & 0xFF;
	return 0;
}

static int s5k4e1_actuator_get_hwparam(void *data, u64 *val)
{
	*val = s5k4e1_hw_params[2];
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(s5k4e1_hwparam,
	s5k4e1_actuator_get_hwparam,
	s5k4e1_actuator_set_hwparam,
	"%llu\n");

static int s5k4e1_actuator_debug_init(void)
{
	struct dentry *debugfs_base = debugfs_create_dir("s5k4e1_actuator", NULL);
	if (!debugfs_base)
		return -ENOMEM;

	if (!debugfs_create_file("s5k4e1_delay",
		S_IRUGO | S_IWUSR, debugfs_base, NULL, &s5k4e1_delay))
		return -ENOMEM;

	if (!debugfs_create_file("s5k4e1_jumpparam",
		S_IRUGO | S_IWUSR, debugfs_base, NULL, &s5k4e1_jumpparam))
		return -ENOMEM;

	if (!debugfs_create_file("s5k4e1_hwparam",
		S_IRUGO | S_IWUSR, debugfs_base, NULL, &s5k4e1_hwparam))
		return -ENOMEM;

	return 0;
}
subsys_initcall(s5k4e1_i2c_add_driver);
MODULE_DESCRIPTION("S5K4E1 actuator");
MODULE_LICENSE("GPL v2");
