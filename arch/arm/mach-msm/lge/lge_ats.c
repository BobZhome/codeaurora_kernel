/* arch/arm/mach-msm/lge_ats.c
 *
 * Copyright (C) 2008 LGE, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <mach/msm_rpcrouter.h>
#include <linux/lge_alohag_at.h>
#include <mach/board_lge.h>
#include "lge_ats.h"

/* Ats server definitions. */

#define ATS_APPS_APISPROG		0x30000006
#define ATS_APPS_APISVERS		0

#define ONCRPC_LGE_ATCMD_ATS_PROC 3
#define ONCRPC_LGE_ATCMD_ATS_ETA_PROC 6

struct ats_data {
	struct atcmd_dev *atdev;
<<<<<<< HEAD
	int (*handle_atcmd) (struct msm_rpc_server * server,
			     struct rpc_request_hdr * req, unsigned len,
			     void (*update_atcmd_state) (char *cmd, int state));
	int (*handle_atcmd_eta) (struct msm_rpc_server * server,
				 struct rpc_request_hdr * req, unsigned len);
=======
	int (*handle_atcmd) (struct msm_rpc_server *server,
						 struct rpc_request_hdr *req, unsigned len,
						 void (*update_atcmd_state)(char *cmd, int state) );
	int (*handle_atcmd_eta) (struct msm_rpc_server *server,
							 struct rpc_request_hdr *req, unsigned len);
>>>>>>> vendor-vs660-froyo
	void (*update_atcmd_state) (char *cmd, int state);
};

struct ats_data lge_ats_data;

static void lge_ats_update_atcmd_state(char *cmd, int state)
{
#if defined (CONFIG_LGE_AT_CMD_DEVICE)
	struct ats_data *data = &lge_ats_data;

<<<<<<< HEAD
	if (!data->atdev)
		data->atdev = atcmd_get_dev();
	if (data->atdev)
=======
	if(!data->atdev)
		data->atdev = atcmd_get_dev();
	if(data->atdev)
>>>>>>> vendor-vs660-froyo
		update_atcmd_state(data->atdev, cmd, state);
#endif
}

static int handle_ats_rpc_call(struct msm_rpc_server *server,
<<<<<<< HEAD
			       struct rpc_request_hdr *req, unsigned len)
{
	struct ats_data *data = &lge_ats_data;

	switch (req->procedure) {
	case ONCRPC_LGE_ATCMD_ATS_ETA_PROC:
		printk(KERN_INFO "%s: ONCRPC_LGE_ATCMD_ATS_ETA_PROC\n",
		       __func__);
		if (data->handle_atcmd_eta)
			return data->handle_atcmd_eta(server, req, len);
		break;
	case ONCRPC_LGE_ATCMD_ATS_PROC:
		printk(KERN_INFO "%s: ONCRPC_LGE_ATCMD_ATS_PROC\n", __func__);
		if (data->handle_atcmd)
			return data->handle_atcmd(server, req, len,
						  data->update_atcmd_state);
		break;
	default:
		return -ENODEV;
=======
							   struct rpc_request_hdr *req, unsigned len)
{
	struct ats_data *data = &lge_ats_data;

	switch (req->procedure)
	{
		case ONCRPC_LGE_ATCMD_ATS_ETA_PROC:
			printk(KERN_INFO"%s: ONCRPC_LGE_ATCMD_ATS_ETA_PROC\n", __func__);
			if(data->handle_atcmd_eta)
				return data->handle_atcmd_eta(server, req, len);
			break;
		case ONCRPC_LGE_ATCMD_ATS_PROC:
			printk(KERN_INFO"%s: ONCRPC_LGE_ATCMD_ATS_PROC\n", __func__);
			if(data->handle_atcmd)
				return data->handle_atcmd(server, req, len, data->update_atcmd_state);
			break;
		default:
			return -ENODEV;
>>>>>>> vendor-vs660-froyo
	}

	return 0;
}

#ifdef CONFIG_LGE_AT_CMD_DEVICE
static struct atcmd_platform_data ats_atcmd_pdata = {
	.name = "alohag_atcmd",
};

static struct platform_device ats_atcmd_device = {
	.name = "alohag_atcmd",
	.id = -1,
<<<<<<< HEAD
	.dev = {
		.platform_data = &ats_atcmd_pdata
	},
};
=======
	.dev    = {
		.platform_data = &ats_atcmd_pdata
	},
}; 
>>>>>>> vendor-vs660-froyo
#endif

#ifdef CONFIG_LGE_ATS_INPUT_DEVICE
static struct platform_device ats_input_device = {
	.name = "ats_input",
};
#endif

static struct msm_rpc_server ats_rpc_server = {
	.prog = ATS_APPS_APISPROG,
	.vers = ATS_APPS_APISVERS,
	.rpc_call = handle_ats_rpc_call,
};

static int __init lge_ats_init(void)
{
	int err;

<<<<<<< HEAD
	if ((err = msm_rpc_create_server(&ats_rpc_server)) != 0) {
		printk(KERN_ERR
		       "%s: Error during creating rpc server for ats atcmd\n",
		       __func__);
=======
	if((err = msm_rpc_create_server(&ats_rpc_server)) != 0) {
		printk(KERN_ERR"%s: Error during creating rpc server for ats atcmd\n", __func__);
>>>>>>> vendor-vs660-froyo
		return err;
	}

	lge_ats_data.handle_atcmd = lge_ats_handle_atcmd;
	lge_ats_data.handle_atcmd_eta = lge_ats_handle_atcmd_eta;
	lge_ats_data.update_atcmd_state = lge_ats_update_atcmd_state;

#ifdef CONFIG_LGE_AT_CMD_DEVICE
	platform_device_register(&ats_atcmd_device);
#endif

#ifdef CONFIG_LGE_ATS_INPUT_DEVICE
	platform_device_register(&ats_input_device);
#endif

	return err;
}

module_init(lge_ats_init);
<<<<<<< HEAD
=======

>>>>>>> vendor-vs660-froyo
