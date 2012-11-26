/* Copyright (c) 2010 - 2011, Code Aurora Forum. All rights reserved.
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
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
#ifndef _MACH_QDSP5_V2_AUDIO_ACDB_DEF_H
#define _MACH_QDSP5_V2_AUDIO_ACDB_DEF_H

#ifndef FEATURE_KYOCERA_MSND
#include <mach/custmsnd.h>
#endif

/* Define ACDB device ID */
#define ACDB_ID_HANDSET_SPKR				1
#define ACDB_ID_HANDSET_MIC				2
#define ACDB_ID_HEADSET_MIC				3
#define ACDB_ID_HEADSET_SPKR_MONO			4
#define ACDB_ID_HEADSET_SPKR_STEREO			5
#define ACDB_ID_SPKR_PHONE_MIC				6
#define ACDB_ID_SPKR_PHONE_MONO				7
#define ACDB_ID_SPKR_PHONE_STEREO			8
#define ACDB_ID_BT_SCO_MIC				9
#define ACDB_ID_BT_SCO_SPKR				0x0A
#define ACDB_ID_BT_A2DP_SPKR				0x0B
#define ACDB_ID_BT_A2DP_TX				0x10
#define ACDB_ID_TTY_HEADSET_MIC				0x0C
#define ACDB_ID_TTY_HEADSET_SPKR			0x0D
#define ACDB_ID_HEADSET_MONO_PLUS_SPKR_MONO_RX		0x11
#define ACDB_ID_HEADSET_STEREO_PLUS_SPKR_STEREO_RX	0x14
#define ACDB_ID_FM_TX_LOOPBACK				0x17
#define ACDB_ID_FM_TX					0x18
#define ACDB_ID_LP_FM_SPKR_PHONE_STEREO_RX		0x19
#define ACDB_ID_LP_FM_HEADSET_SPKR_STEREO_RX		0x1A
#define ACDB_ID_I2S_RX					0x20
#define ACDB_ID_SPKR_PHONE_MIC_BROADSIDE		0x2B
#define ACDB_ID_HANDSET_MIC_BROADSIDE			0x2C
#define ACDB_ID_SPKR_PHONE_MIC_ENDFIRE			0x2D
#define ACDB_ID_HANDSET_MIC_ENDFIRE			0x2E
#define ACDB_ID_I2S_TX					0x30
#define ACDB_ID_HDMI					0x40
#define ACDB_ID_FM_RX					0x4F
#ifdef FEATURE_KYOCERA_MSND
#define ACDB_ID_HAC_RX					0x57
#define ACDB_ID_BT_SCO_MIC_EC_OFF				0x58
#define ACDB_ID_BT_SCO_SPKR_EC_OFF				0x59
#define ACDB_ID_HANDSET_SPKR_VOIP				0x5A
#define ACDB_ID_HANDSET_MIC_VOIP				0x5B
#define ACDB_ID_SPKR_PHONE_MONO_VOIP				0x5C
#define ACDB_ID_SPKR_PHONE_MIC_VOIP				0x5D
#define ACDB_ID_HEADSET_SPKR_STEREO_VOIP				0x5E
#define ACDB_ID_HEADSET_MIC_VOIP				0x5F
#define ACDB_ID_BT_SCO_SPKR_VOIP				0x60
#define ACDB_ID_BT_SCO_MIC_VOIP				0x61
#define ACDB_ID_BT_SCO_SPKR_EC_OFF_VOIP				0x62
#define ACDB_ID_BT_SCO_MIC_EC_OFF_VOIP				0x63
#define ACDB_ID_HAC_RX_VOIP				0x64
#define ACDB_ID_SPKR_PHONE_MONO_OPEN				0x65
#define ACDB_ID_SPKR_PHONE_MIC_OPEN				0x66
#define ACDB_ID_SPKR_PHONE_MONO_VOIP_OPEN				0x67
#define ACDB_ID_SPKR_PHONE_MIC_VOIP_OPEN				0x68
#define ACDB_ID_MAX                                 ACDB_ID_SPKR_PHONE_MIC_VOIP_OPEN
#else
/*Replace the max device ID,if any new device is added Specific to RTC only*/
#define ACDB_ID_MAX                                 ACDB_ID_FM_RX
#endif

/* ID used for virtual devices */
#define PSEUDO_ACDB_ID 					0xFFFF

#endif /* _MACH_QDSP5_V2_AUDIO_ACDB_DEF_H */
