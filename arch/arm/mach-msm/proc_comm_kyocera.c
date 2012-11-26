/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C)2012 KYOCERA Corporation
 */







#include "proc_comm.h"


int proc_comm_rpc_apps_to_modem(unsigned sub_cmd, unsigned *data)
{
	return msm_proc_comm(PCOM_CUSTOMER_CMD2, &sub_cmd, data);
}

