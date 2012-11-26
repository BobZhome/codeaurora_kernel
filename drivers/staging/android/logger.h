/* include/linux/logger.h
 *
 * Copyright (C) 2007-2008 Google, Inc.
 * Author: Robert Love <rlove@android.com>
 *
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
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

#ifndef _LINUX_LOGGER_H
#define _LINUX_LOGGER_H

#include <linux/types.h>
#include <linux/ioctl.h>

#include <mach/msm_iomap.h>

struct logger_entry {
	__u16		len;	/* length of the payload */
	__u16		__pad;	/* no matter what, we get 2 bytes of padding */
	__s32		pid;	/* generating process's pid */
	__s32		tid;	/* generating process's tid */
	__s32		sec;	/* seconds since Epoch */
	__s32		nsec;	/* nanoseconds */
	char		msg[0];	/* the entry's payload */
};

#define LOGGER_LOG_RADIO	"log_radio"	/* radio-related messages */
#define LOGGER_LOG_EVENTS	"log_events"	/* system/hardware events */
#define LOGGER_LOG_SYSTEM	"log_system"	/* system/framework messages */
#define LOGGER_LOG_MAIN		"log_main"	/* everything else */

#define LOGGER_ENTRY_MAX_LEN		(4*1024)
#define LOGGER_ENTRY_MAX_PAYLOAD	\
	(LOGGER_ENTRY_MAX_LEN - sizeof(struct logger_entry))

#define __LOGGERIO	0xAE

#define LOGGER_GET_LOG_BUF_SIZE		_IO(__LOGGERIO, 1) /* size of log */
#define LOGGER_GET_LOG_LEN		_IO(__LOGGERIO, 2) /* used log len */
#define LOGGER_GET_NEXT_ENTRY_LEN	_IO(__LOGGERIO, 3) /* next entry len */
#define LOGGER_FLUSH_LOG		_IO(__LOGGERIO, 4) /* flush log */

/*****************************************************************************************/
/* Refer from kcjlogger.c to this definition. Be careful when you change.                */
/*****************************************************************************************/
struct logger_log_info {
        size_t                  w_off;          /* write poiner */
        size_t                  head;           /* log top pointer */
};

#define RAM_CONSOLE_SIZE        (CONFIG_ANDROID_RAM_CONSOLE_EARLY_SIZE)

#define LOG_MAGIC_SIZE          (16)
#define LOG_MAIN_SIZE           (512 * 1024)
#define LOG_RADIO_SIZE          (512 * 1024)
#define LOG_SYSTEM_SIZE         (512 * 1024)
#define LOG_EVENTS_SIZE         (256 * 1024)
#define CONTROL_INFO_SIZE       (256)

#define STARTADDRESS            (MSM_UNINIT_RAM_BASE + RAM_CONSOLE_SIZE)
#define ADDR_CONTROL_INFO       (STARTADDRESS)
#define ADDR_LOG_MAIN           (ADDR_CONTROL_INFO + CONTROL_INFO_SIZE)
#define ADDR_LOG_EVENTS         (ADDR_LOG_MAIN + LOG_MAIN_SIZE)
#define ADDR_LOG_RADIO          (ADDR_LOG_EVENTS + LOG_EVENTS_SIZE)
#define ADDR_LOG_SYSTEM         (ADDR_LOG_RADIO + LOG_RADIO_SIZE)

#define LOGGER_INFO_SIZE        (sizeof(struct logger_log_info))
#define ADDR_LOGGER_INFO_MAIN   (ADDR_CONTROL_INFO + LOG_MAGIC_SIZE)
#define ADDR_LOGGER_INFO_EVENTS (ADDR_LOGGER_INFO_MAIN + LOGGER_INFO_SIZE)
#define ADDR_LOGGER_INFO_RADIO  (ADDR_LOGGER_INFO_EVENTS + LOGGER_INFO_SIZE)
#define ADDR_LOGGER_INFO_SYSTEM (ADDR_LOGGER_INFO_RADIO + LOGGER_INFO_SIZE)

#define DEFINE_LOGGER_DEVICE(VAR, ADDR, NAME, SIZE, LOGGER_INFO, LOGGER_FOPS) \
static struct logger_log VAR = { \
        .buffer = ADDR, \
        .misc = { \
                .minor = MISC_DYNAMIC_MINOR, \
                .name = NAME, \
                .fops = LOGGER_FOPS, \
                .parent = NULL, \
        }, \
        .wq = __WAIT_QUEUE_HEAD_INITIALIZER(VAR .wq), \
        .readers = LIST_HEAD_INIT(VAR .readers), \
        .mutex = __MUTEX_INITIALIZER(VAR .mutex), \
        .w_off = 0, \
        .head = 0, \
        .size = SIZE, \
        .log_info = LOGGER_INFO, \
};

#endif /* _LINUX_LOGGER_H */
