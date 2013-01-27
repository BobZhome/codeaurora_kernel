/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
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

#include <linux/slab.h>
#include <linux/init.h>
#include <linux/uaccess.h>
#include <linux/diagchar.h>
#include <linux/sched.h>
#include <linux/err.h>
#include <linux/ratelimit.h>
#include <linux/workqueue.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <asm/current.h>
#ifdef CONFIG_DIAG_OVER_USB
#include <mach/usbdiag.h>
#endif
#include "diagchar_hdlc.h"
#include "diagmem.h"
#include "diagchar.h"
#include "diagfwd.h"
#include "diagfwd_hsic.h"

#ifdef CONFIG_LGE_USB_DIAG_DISABLE
#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/fs.h>
#endif //#ifdef CONFIG_LGE_USB_DIAG_DISABLE

#ifdef CONFIG_LGE_USB_MDM_DIAG_DISABLE
int mdm_diag_enable = 1;
#endif //#ifdef CONFIG_LGE_USB_MDM_DIAG_DISABLE

static void diag_read_hsic_work_fn(struct work_struct *work)
{
#ifdef CONFIG_LGE_USB_MDM_DIAG_DISABLE
	if(mdm_diag_enable == 0)
	{
		pr_debug("diag: [diag_read_hsic_work_fn] mdm_diag diabled\n");
		return;
	}
#endif //#ifdef CONFIG_LGE_USB_MDM_DIAG_DISABLE

#define N_MDM_WRITE	8
#define N_MDM_READ	1

#define READ_HSIC_BUF_SIZE 2048

#define NUM_HSIC_BUF_TBL_ENTRIES N_MDM_WRITE

static void diag_read_hsic_work_fn(struct work_struct *work)
{
	unsigned char *buf_in_hsic = NULL;
	int num_reads_submitted = 0;
	int err = 0;
	int write_ptrs_available;

	if (!driver->hsic_ch) {
		pr_err("DIAG in %s: driver->hsic_ch == 0\n", __func__);
		return;
	}

	/*
	 * Determine the current number of available buffers for writing after
	 * reading from the HSIC has completed.
	 */
	if (driver->logging_mode == MEMORY_DEVICE_MODE)
		write_ptrs_available = driver->poolsize_hsic_write -
					driver->num_hsic_buf_tbl_entries;
	else
		write_ptrs_available = driver->poolsize_hsic_write -
					driver->count_hsic_write_pool;

	/*
	 * Queue up a read on the HSIC for all available buffers in the
	 * pool, exhausting the pool.
	 */
	do {
		/*
		 * If no more write buffers are available,
		 * stop queuing reads
		 */
		if (write_ptrs_available <= 0)
			break;

		write_ptrs_available--;

		buf_in_hsic = diagmem_alloc(driver, READ_HSIC_BUF_SIZE,
							POOL_TYPE_HSIC);
		if (buf_in_hsic) {
			/*
			 * Initiate the read from the hsic.  The hsic read is
			 * asynchronous.  Once the read is complete the read
			 * callback function will be called.
			 */
			pr_debug("diag: read from HSIC\n");
			num_reads_submitted++;
			err = diag_bridge_read((char *)buf_in_hsic,
							READ_HSIC_BUF_SIZE);
			if (err) {
				num_reads_submitted--;

				/* Return the buffer to the pool */
				diagmem_free(driver, buf_in_hsic,
						POOL_TYPE_HSIC);

				pr_err_ratelimited("diag: Error initiating HSIC read, err: %d\n",
					err);
				/*
				 * An error occurred, discontinue queuing
				 * reads
				 */
				break;
			}
		}
	} while (buf_in_hsic);

	/*
	 * If there are no buffers available or for some reason there
	 * was no hsic data, and if no unrecoverable error occurred
	 * (-ENODEV is an unrecoverable error), then set up the next read
	 */
	if ((num_reads_submitted == 0) && (err != -ENODEV))
		queue_work(driver->diag_hsic_wq,
				 &driver->diag_read_hsic_work);
}

static void diag_hsic_read_complete_callback(void *ctxt, char *buf,
					int buf_size, int actual_size)
{
	int err = -2;

	if (!driver->hsic_ch) {
		/*
		 * The hsic channel is closed. Return the buffer to
		 * the pool.  Do not send it on.
		 */
		diagmem_free(driver, buf, POOL_TYPE_HSIC);
		pr_debug("diag: In %s: driver->hsic_ch == 0, actual_size: %d\n",
			__func__, actual_size);
		return;
	}

	/* Note that zero length is valid and still needs to be sent */
	if (actual_size >= 0) {
		if (!buf) {
			pr_err("diag: Out of diagmem for HSIC\n");
		} else {
			/*
			 * Send data in buf to be written on the
			 * appropriate device, e.g. USB MDM channel
			 */
			driver->write_len_mdm = actual_size;
			err = diag_device_write((void *)buf, HSIC_DATA, NULL);
			/* If an error, return buffer to the pool */
			if (err) {
				diagmem_free(driver, buf, POOL_TYPE_HSIC);
				pr_err("diag: In %s, error calling diag_device_write, err: %d\n",
					__func__, err);
			}
		}
	} else {
		/*
		 * The buffer has an error status associated with it. Do not
		 * pass it on. Note that -ENOENT is sent when the diag bridge
		 * is closed.
		 */
		diagmem_free(driver, buf, POOL_TYPE_HSIC);
		pr_debug("diag: In %s: error status: %d\n", __func__,
			actual_size);
	}

	/*
	 * If for some reason there was no hsic data to write to the
	 * mdm channel, set up another read
	 */
	if (err &&
		((driver->logging_mode == MEMORY_DEVICE_MODE) ||
		(driver->usb_mdm_connected && !driver->hsic_suspend))) {
		queue_work(driver->diag_hsic_wq,
				 &driver->diag_read_hsic_work);
	}
}

static void diag_hsic_write_complete_callback(void *ctxt, char *buf,
					int buf_size, int actual_size)
{
	/* The write of the data to the HSIC bridge is complete */
	driver->in_busy_hsic_write = 0;

	if (!driver->hsic_ch) {
		pr_err("DIAG in %s: driver->hsic_ch == 0\n", __func__);
		return;
	}

	if (actual_size < 0)
		pr_err("DIAG in %s: actual_size: %d\n", __func__, actual_size);

	if (driver->usb_mdm_connected)
		queue_work(driver->diag_hsic_wq, &driver->diag_read_mdm_work);
}

static int diag_hsic_suspend(void *ctxt)
{
	pr_debug("diag: hsic_suspend\n");

	/* Don't allow suspend if a write in the HSIC is in progress */
	if (driver->in_busy_hsic_write)
		return -EBUSY;

	/* Don't allow suspend if in MEMORY_DEVICE_MODE */
	if (driver->logging_mode == MEMORY_DEVICE_MODE)
		return -EBUSY;

	driver->hsic_suspend = 1;

	return 0;
}

static void diag_hsic_resume(void *ctxt)
{
	pr_debug("diag: hsic_resume\n");
	driver->hsic_suspend = 0;

	if ((driver->logging_mode == MEMORY_DEVICE_MODE) ||
				(driver->usb_mdm_connected))
		queue_work(driver->diag_hsic_wq,
			 &driver->diag_read_hsic_work);
}

static struct diag_bridge_ops hsic_diag_bridge_ops = {
	.ctxt = NULL,
	.read_complete_cb = diag_hsic_read_complete_callback,
	.write_complete_cb = diag_hsic_write_complete_callback,
	.suspend = diag_hsic_suspend,
	.resume = diag_hsic_resume,
};

static int diag_hsic_close(void)
{
	if (driver->hsic_device_enabled) {
		driver->hsic_ch = 0;
		if (driver->hsic_device_opened) {
			driver->hsic_device_opened = 0;
			diag_bridge_close();
		}
		pr_debug("diag: in %s: closed successfully\n", __func__);
	} else {
		pr_debug("diag: in %s: already closed\n", __func__);
	}

	return 0;
}

/* diagfwd_cancel_hsic is called to cancel outstanding read/writes */
int diagfwd_cancel_hsic(void)
{
	int err;

	if (driver->hsic_device_enabled) {
		if (driver->hsic_device_opened) {
			driver->hsic_ch = 0;
			driver->hsic_device_opened = 0;
			diag_bridge_close();
			err = diag_bridge_open(&hsic_diag_bridge_ops);
			if (err) {
				pr_err("diag: HSIC channel open error: %d\n",
					err);
			} else {
				pr_debug("diag: opened HSIC channel\n");
				driver->hsic_device_opened = 1;
				driver->hsic_ch = 1;
			}
		}
	}

	return 0;
}

/* diagfwd_connect_hsic is called when the USB mdm channel is connected */
int diagfwd_connect_hsic(int process_cable)
{
	int err;

	pr_debug("DIAG in %s\n", __func__);

	/* If the usb cable is being connected */
	if (process_cable) {
		err = usb_diag_alloc_req(driver->mdm_ch, N_MDM_WRITE,
			N_MDM_READ);
		if (err)
			pr_err("DIAG: unable to alloc USB req on mdm"
				" ch err:%d\n", err);

		driver->usb_mdm_connected = 1;
	}

	if (driver->hsic_device_enabled) {
		driver->in_busy_hsic_read_on_device = 0;
		driver->in_busy_hsic_write = 0;
	}

	/* If the hsic (diag_bridge) platform device is not open */
	if (driver->hsic_device_enabled) {
		if (!driver->hsic_device_opened) {
			err = diag_bridge_open(&hsic_diag_bridge_ops);
			if (err) {
				pr_err("DIAG: HSIC channel open error: %d\n",
					err);
			} else {
				pr_debug("DIAG: opened HSIC channel\n");
				driver->hsic_device_opened = 1;
			}
		} else {
			pr_debug("DIAG: HSIC channel already open\n");
		}

		/*
		 * Turn on communication over usb mdm and hsic, if the hsic
		 * device driver is enabled and opened
		 */
		if (driver->hsic_device_opened)
			driver->hsic_ch = 1;

		/* Poll USB mdm channel to check for data */
		if (driver->logging_mode == USB_MODE)
			queue_work(driver->diag_hsic_wq,
					&driver->diag_read_mdm_work);

		/* Poll HSIC channel to check for data */
		queue_work(driver->diag_hsic_wq, &driver->diag_read_hsic_work);
	} else {
		/* The hsic device driver has not yet been enabled */
		pr_info("DIAG: HSIC channel not yet enabled\n");
	}

	return 0;
}

/*
 * diagfwd_disconnect_hsic is called when the USB mdm channel
 * is disconnected
 */
int diagfwd_disconnect_hsic(int process_cable)
{
	pr_debug("diag: In %s, process_cable: %d\n", __func__, process_cable);

	/* If the usb cable is being disconnected */
	if (process_cable) {
		driver->usb_mdm_connected = 0;
		usb_diag_free_req(driver->mdm_ch);
	}

	if (driver->logging_mode != MEMORY_DEVICE_MODE) {
		if (driver->hsic_device_enabled) {
			driver->in_busy_hsic_read_on_device = 1;
			driver->in_busy_hsic_write = 1;
			/* Turn off communication over usb mdm and hsic */
			return diag_hsic_close();
		}
	}
	return 0;
}

/*
 * diagfwd_write_complete_hsic is called after the asynchronous
 * usb_diag_write() on mdm channel is complete
 */
int diagfwd_write_complete_hsic(struct diag_request *diag_write_ptr)
{
	unsigned char *buf = (diag_write_ptr) ? diag_write_ptr->buf : NULL;

	if (buf) {
		/* Return buffers to their pools */
		diagmem_free(driver, (unsigned char *)buf, POOL_TYPE_HSIC);
		diagmem_free(driver, (unsigned char *)diag_write_ptr,
							POOL_TYPE_HSIC_WRITE);
	}

	if (!driver->hsic_ch) {
		pr_err("diag: In %s: driver->hsic_ch == 0\n", __func__);
		return 0;
	}

	/* Read data from the hsic */
	queue_work(driver->diag_hsic_wq, &driver->diag_read_hsic_work);

	return 0;
}

/* Called after the asychronous usb_diag_read() on mdm channel is complete */
static int diagfwd_read_complete_hsic(struct diag_request *diag_read_ptr)
{
	/* The read of the usb driver on the mdm (not hsic) has completed */
	driver->in_busy_hsic_read_on_device = 0;
	driver->read_len_mdm = diag_read_ptr->actual;

	if (!driver->hsic_ch) {
		pr_err("DIAG in %s: driver->hsic_ch == 0\n", __func__);
		return 0;
	}

	/*
	 * The read of the usb driver on the mdm channel has completed.
	 * If there is no write on the hsic in progress, check if the
	 * read has data to pass on to the hsic. If so, pass the usb
	 * mdm data on to the hsic.
	 */
	if (!driver->in_busy_hsic_write && driver->usb_buf_mdm_out &&
		(driver->read_len_mdm > 0)) {

		/*
		 * Initiate the hsic write. The hsic write is
		 * asynchronous. When complete the write
		 * complete callback function will be called
		 */
		int err;
		driver->in_busy_hsic_write = 1;
		err = diag_bridge_write(driver->usb_buf_mdm_out,
					driver->read_len_mdm);
		if (err) {
			pr_err_ratelimited("diag: mdm data on hsic write err: %d\n",
					err);
			/*
			 * If the error is recoverable, then clear
			 * the write flag, so we will resubmit a
			 * write on the next frame.  Otherwise, don't
			 * resubmit a write on the next frame.
			 */
			if ((-ENODEV) != err)
				driver->in_busy_hsic_write = 0;
		}
	}

	/*
	 * If there is no write of the usb mdm data on the
	 * hsic channel
	 */
	if (!driver->in_busy_hsic_write)
		queue_work(driver->diag_hsic_wq, &driver->diag_read_mdm_work);

	return 0;
}

static void diagfwd_hsic_notifier(void *priv, unsigned event,
					struct diag_request *d_req)
{
	switch (event) {
	case USB_DIAG_CONNECT:
		diagfwd_connect_hsic(1);
		break;
	case USB_DIAG_DISCONNECT:
		queue_work(driver->diag_hsic_wq, &driver->diag_disconnect_work);
		break;
	case USB_DIAG_READ_DONE:
		queue_work(driver->diag_hsic_wq,
				&driver->diag_usb_read_complete_work);
		break;
	case USB_DIAG_WRITE_DONE:
		if (driver->hsic_device_enabled)
			diagfwd_write_complete_hsic(d_req);
		break;
	default:
		pr_err("DIAG in %s: Unknown event from USB diag:%u\n",
			__func__, event);
		break;
	}
}

static void diag_usb_read_complete_fn(struct work_struct *w)
{
	diagfwd_read_complete_hsic(driver->usb_read_mdm_ptr);
}

static void diag_disconnect_work_fn(struct work_struct *w)
{
	diagfwd_disconnect_hsic(1);
}

static void diag_read_mdm_work_fn(struct work_struct *work)
{

#ifdef CONFIG_LGE_USB_MDM_DIAG_DISABLE
	if(mdm_diag_enable == 0)
	{
		pr_debug("diag: [diag_read_mdm_work_fn] mdm_diag diabled\n");
		return;
	}
#endif //#ifdef CONFIG_LGE_USB_MDM_DIAG_DISABLE

	if (!driver->hsic_ch) {
		pr_err("DIAG in %s: driver->hsic_ch == 0\n", __func__);
		return;
	}

	/*
	 * If there is no data being read from the usb mdm channel
	 * and there is no mdm channel data currently being written
	 * to the hsic
	 */
	if (!driver->in_busy_hsic_read_on_device &&
				 !driver->in_busy_hsic_write) {
		APPEND_DEBUG('x');

		/* Setup the next read from usb mdm channel */
		driver->in_busy_hsic_read_on_device = 1;
		driver->usb_read_mdm_ptr->buf = driver->usb_buf_mdm_out;
		driver->usb_read_mdm_ptr->length = USB_MAX_OUT_BUF;
		usb_diag_read(driver->mdm_ch, driver->usb_read_mdm_ptr);
		APPEND_DEBUG('y');
	}

	/*
	 * If for some reason there was no mdm channel read initiated,
	 * queue up the reading of data from the mdm channel
	 */
	if (!driver->in_busy_hsic_read_on_device)
		queue_work(driver->diag_hsic_wq, &driver->diag_read_mdm_work);
}

static int diag_hsic_probe(struct platform_device *pdev)
{
	int err = 0;
	pr_debug("diag: in %s\n", __func__);
	if (!driver->hsic_device_enabled) {
		driver->read_len_mdm = 0;
		if (driver->buf_in_hsic == NULL)
			driver->buf_in_hsic = kzalloc(IN_BUF_SIZE, GFP_KERNEL);
		if (driver->buf_in_hsic == NULL)
			goto err;
		if (driver->usb_buf_mdm_out  == NULL)
			driver->usb_buf_mdm_out = kzalloc(USB_MAX_OUT_BUF,
								 GFP_KERNEL);
		if (driver->usb_buf_mdm_out == NULL)
			goto err;
		if (driver->usb_read_mdm_ptr == NULL)
			driver->usb_read_mdm_ptr = kzalloc(
			sizeof(struct diag_request), GFP_KERNEL);
		if (driver->usb_read_mdm_ptr == NULL)
			goto err;
#ifdef CONFIG_DIAG_OVER_USB
		INIT_WORK(&(driver->diag_read_mdm_work), diag_read_mdm_work_fn);
#endif
		diagmem_hsic_init(driver);
		INIT_WORK(&(driver->diag_read_hsic_work),
						 diag_read_hsic_work_fn);
		driver->hsic_device_enabled = 1;
	}

	/*
	 * The probe function was called after the usb was connected
	 * on the legacy channel OR ODL is turned on. Communication over usb
	 * mdm and hsic needs to be turned on.
	 */
	if (driver->usb_mdm_connected || (driver->logging_mode ==
							 MEMORY_DEVICE_MODE)) {
		/* The hsic (diag_bridge) platform device driver is enabled */
		err = diag_bridge_open(&hsic_diag_bridge_ops);
		if (err) {
			pr_err("diag: could not open HSIC, err: %d\n", err);
			driver->hsic_device_opened = 0;
			return err;
		}

		pr_info("diag: opened HSIC channel\n");
		driver->hsic_device_opened = 1;
		driver->hsic_ch = 1;

		driver->in_busy_hsic_read_on_device = 0;
		driver->in_busy_hsic_write = 0;

		if (driver->usb_mdm_connected) {
			/* Poll USB mdm channel to check for data */
			queue_work(driver->diag_hsic_wq,
					 &driver->diag_read_mdm_work);
		}

		/* Poll HSIC channel to check for data */
		queue_work(driver->diag_hsic_wq, &driver->diag_read_hsic_work);
	}

	return err;
err:
	pr_err("DIAG could not initialize buf for HSIC\n");
	kfree(driver->buf_in_hsic);
	kfree(driver->usb_buf_mdm_out);
	kfree(driver->usb_read_mdm_ptr);
	if (driver->diag_hsic_wq)
		destroy_workqueue(driver->diag_hsic_wq);

	return -ENOMEM;
}

static int diag_hsic_remove(struct platform_device *pdev)
{
	pr_debug("diag: %s called\n", __func__);
	diag_hsic_close();
	return 0;
}

static int diagfwd_hsic_runtime_suspend(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: suspending...\n");
	return 0;
}

static int diagfwd_hsic_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: resuming...\n");
	return 0;
}

static const struct dev_pm_ops diagfwd_hsic_dev_pm_ops = {
	.runtime_suspend = diagfwd_hsic_runtime_suspend,
	.runtime_resume = diagfwd_hsic_runtime_resume,
};

static struct platform_driver msm_hsic_ch_driver = {
	.probe = diag_hsic_probe,
	.remove = diag_hsic_remove,
	.driver = {
		   .name = "diag_bridge",
		   .owner = THIS_MODULE,
		   .pm   = &diagfwd_hsic_dev_pm_ops,
		   },
};

void diagfwd_hsic_init(void)
{
	int ret;

	pr_debug("DIAG in %s\n", __func__);
	driver->diag_hsic_wq = create_singlethread_workqueue("diag_hsic_wq");
	driver->write_len_mdm = 0;
	driver->num_hsic_buf_tbl_entries = 0;
	if (driver->hsic_buf_tbl == NULL)
		driver->hsic_buf_tbl = kzalloc(NUM_HSIC_BUF_TBL_ENTRIES *
				sizeof(struct diag_write_device), GFP_KERNEL);
	if (driver->hsic_buf_tbl == NULL)
		goto err;

	driver->count_hsic_pool = 0;
	driver->count_hsic_write_pool = 0;
	driver->itemsize_hsic = READ_HSIC_BUF_SIZE;
	driver->poolsize_hsic = N_MDM_WRITE;
	driver->itemsize_hsic_write = sizeof(struct diag_request);
	driver->poolsize_hsic_write = N_MDM_WRITE;

	INIT_WORK(&(driver->diag_disconnect_work), diag_disconnect_work_fn);
	INIT_WORK(&(driver->diag_usb_read_complete_work),
			diag_usb_read_complete_fn);

#ifdef CONFIG_DIAG_OVER_USB
	driver->mdm_ch = usb_diag_open(DIAG_MDM, driver, diagfwd_hsic_notifier);
	if (IS_ERR(driver->mdm_ch)) {
		pr_err("DIAG Unable to open USB diag MDM channel\n");
		goto err;
	}
#endif
	ret = platform_driver_register(&msm_hsic_ch_driver);
	if (ret)
		pr_err("DIAG could not register HSIC device, ret: %d\n", ret);
	else
		driver->hsic_initialized = 1;

	return;
err:
	pr_err("diag: Could not initialize for HSIC execution\n");
	kfree(driver->hsic_buf_tbl);

	return;
}

void diagfwd_hsic_exit(void)
{
	pr_debug("diag: in %s\n", __func__);

	if (driver->hsic_initialized)
		diag_hsic_close();
	diagmem_exit(driver, POOL_TYPE_ALL);
	/* destroy USB MDM specific variables */
#ifdef CONFIG_DIAG_OVER_USB
	if (driver->usb_mdm_connected)
		usb_diag_free_req(driver->mdm_ch);
#endif
	platform_driver_unregister(&msm_hsic_ch_driver);
#ifdef CONFIG_DIAG_OVER_USB
	usb_diag_close(driver->mdm_ch);
#endif
	kfree(driver->buf_in_hsic);
	kfree(driver->usb_buf_mdm_out);
	kfree(driver->hsic_buf_tbl);
	kfree(driver->usb_read_mdm_ptr);
	destroy_workqueue(driver->diag_hsic_wq);

	driver->hsic_device_enabled = 0;
}

#ifdef CONFIG_LGE_USB_DIAG_DISABLE

/* 2011.12.07 jaeho.cho@lge.com diag enable/disable*/

#define DIAG_LOCKED "0"
#define DIAG_UNLOCKED "1"

void set_mdm_diag_enable(int enable)
{
	mdm_diag_enable = enable;
}
EXPORT_SYMBOL(set_mdm_diag_enable);
int get_mdm_diag_enable(void)
{
	return mdm_diag_enable;
}
EXPORT_SYMBOL(get_mdm_diag_enable);
#endif //#ifdef CONFIG_LGE_USB_MDM_DIAG_DISABLE

#ifdef CONFIG_LGE_USB_DIAG_DISABLE
extern int user_diag_enable;
static const char *diag_storage = "/factory/diaginfo";

int save_diaginfo_toStorage(char ch)
{

	struct file *file;
    int fd = -1;
	loff_t pos = 0;
	mm_segment_t old_fs = 0;

	old_fs = get_fs();
	set_fs(get_ds());

	fd = sys_open(diag_storage, O_WRONLY|O_CREAT, 0644);
	if (fd >= 0) {
	   file = fget(fd);
	   if (file) {
            printk("[save_diaginfo_toStorage] sys_open success\n");
            vfs_write(file, &ch, 1, &pos);
            fput(file);
	   }
	   sys_close(fd);
	}
	else
	{
	//error
			printk("[save_diaginfo_toStorage] sys_open error\n");
			return 0;

	}
	set_fs(old_fs);
	return 1;

}

int get_diaginfo_fromStorge(char* ch)
{
    int fd = -1;
    mm_segment_t old_fs = 0;
    int dwReadLen=0;
    old_fs = get_fs();
    set_fs(get_ds());

		if ((fd = sys_open(diag_storage, O_RDONLY, 0)) < 0){
			//file not exist
			printk("[get_diaginfo_fromStorge] sys_open error\n");
			set_fs(old_fs);
			return 0;
		}

		printk("[get_diaginfo_fromStorge] sys_read before\n");

		dwReadLen = sys_read(fd, ch,1);
		if( dwReadLen == 1 )
		{
		  //success
		  printk("[get_diaginfo_fromStorge] sys_read success\n");
		}
		else
		{
			printk("[get_diaginfo_fromStorge] sys_read error\n");
			sys_close(fd);
			set_fs(old_fs);
			return 0;
		}

        sys_close(fd);
		set_fs(old_fs);

        return 1;
}

int get_usb_lock(void)
{
	//extern int lg_get_usb_lock_state(void);

	#if 0
	if (lg_get_usb_lock_state())
		user_diag_enable = 0;
	else
		user_diag_enable = 1;
	#endif

    return !(user_diag_enable);
}
EXPORT_SYMBOL(get_usb_lock);

void set_usb_lock(int lock)
{
	//extern int lg_set_usb_lock_state(int lock);

	//lg_set_usb_lock_state(lock);

	if (lock)
		user_diag_enable = 0;
	else
		user_diag_enable = 1;
}
EXPORT_SYMBOL(set_usb_lock);

static ssize_t read_diag_enable(struct device *dev, struct device_attribute *attr,
				   char *buf)
{
	int ret = 0, diag_saved_info=0;
	unsigned char string[2]={0};
	printk("[read_diag_enable] called\n");

    if(buf ==NULL)
	{
			printk(KERN_ERR "%s invalid string length \n",__func__);
			ret = 0;
	}

	//getting information from storage

	diag_saved_info=get_diaginfo_fromStorge(string);

    printk("[read_diag_enable] read : %c called\n",string[0]);
	//update
	if(diag_saved_info==1)
	{
		//sscanf(string,"%c",buf);
		if(!strncmp(string, DIAG_UNLOCKED, 1))
		{
			printk("[read_diag_enable] diag unlocked\n");
			set_usb_lock(0);
			set_mdm_diag_enable(1);
                     ret=sprintf(buf, "%d", user_diag_enable);
        }
		else if(!strncmp(string, DIAG_LOCKED, 1))
		{
			printk("[read_diag_enable] diag locked\n");
			set_usb_lock(1);
			set_mdm_diag_enable(0);
			ret=sprintf(buf, "%d", user_diag_enable);
		}
		else
		{
			printk("[read_diag_enable] unknown data\n");
		}
	}
	else
	{//temp
		printk("[read_diag_enable] not stored\n");
		ret = sprintf(buf, "%d", user_diag_enable); // lock info
	}
	// mdm_diag_enable, user_diag_enable

    if(ret < 0)
	{
		printk(KERN_ERR "%s invalid string length \n",__func__);
		ret = 0;
	}

	return ret;
}
static ssize_t write_diag_enable(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t size)
{
    unsigned char string[2];
	printk("[write_diag_enable] called\n");


    sscanf(buf, "%s", string);

	//save information
	save_diaginfo_toStorage(string[0]);


	//update
    if(!strncmp(string, DIAG_UNLOCKED, 1))
    {
		printk("[write_diag_enable] 1 value called\n");

		set_usb_lock(0);
#ifdef CONFIG_LGE_USB_MDM_DIAG_DISABLE
		set_mdm_diag_enable(1);
#endif
    }

	else if(!strncmp(string, DIAG_LOCKED, 1))
	{
		printk("[write_diag_enable] 0 value called\n");
		set_usb_lock(1);
#ifdef CONFIG_LGE_USB_MDM_DIAG_DISABLE
		set_mdm_diag_enable(0);
#endif
	}
	return size;

/* 2011.12.07 jaeho.cho@lge.com diag enable/disable*/
static DEVICE_ATTR(diag_enable, S_IRUGO | S_IWUSR, read_diag_enable, write_diag_enable);

/* 2011.12.07 jaeho.cho@lge.com diag enable/disable*/
int lg_diag_create_file(struct platform_device *pdev)
{
	int ret;
	//int wfd;

/* 2011.12.07 jaeho.cho@lge.com diag enable/disable*/

	ret = device_create_file(&pdev->dev, &dev_attr_diag_enable);
	if (ret) {
		printk( KERN_DEBUG "LG_FW : diag device diag_enable create fail\n");
		device_remove_file(&pdev->dev, &dev_attr_diag_enable);
		return ret;
	}

#if 0
    wfd = sys_open("/sys/devices/platform/lg_diag_cmd/diag_enable", O_WRONLY, 0);
    if (wfd >= 0)
    {
        sys_fchown(wfd, 1000, 1000);
        sys_fchmod(wfd, 0660);
    }
    else
        pr_info("%s: permission denied\n", __func__);
#endif
/* 2011.12.07 jaeho.cho@lge.com diag enable/disable*/
	return ret;
}
EXPORT_SYMBOL(lg_diag_create_file);

int lg_diag_remove_file(struct platform_device *pdev)
{
/* 2011.12.07 jaeho.cho@lge.com diag enable/disable*/

	device_remove_file(&pdev->dev, &dev_attr_diag_enable);

/* 2011.12.07 jaeho.cho@lge.com diag enable/disable*/

	return 0;
}
EXPORT_SYMBOL(lg_diag_remove_file);
#endif //#ifdef CONFIG_LGE_USB_DIAG_DISABLE
