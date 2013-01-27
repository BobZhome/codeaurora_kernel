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

#ifndef MT9M114_H
#define MT9M114_H

#include <linux/types.h>
#include <mach/board.h>

extern struct mt9m114_reg mt9m114_regs;

struct mt9m114_i2c_reg_conf {
	unsigned short waddr;
	unsigned short wdata;
};

enum mt9m114_width {
	BYTE_LEN,
	WORD_LEN,
};

enum mt9m114_test_mode_t {
	TEST_OFF,
	TEST_1,
	TEST_2,
	TEST_3
};

struct mt9m114_reg {

	struct mt9m114_i2c_reg_conf const *init_tbl;
	uint16_t inittbl_size;
	// Preview
	struct mt9m114_i2c_reg_conf const *prev_tbl;
	uint16_t prevtbl_size;
	// Snapshot
	struct mt9m114_i2c_reg_conf const *snap_tbl;
	uint16_t snaptbl_size;
	// Effect
	struct mt9m114_i2c_reg_conf const *effect_default_tbl;
	uint16_t effect_default_tbl_size;
	struct mt9m114_i2c_reg_conf const *effect_mono_tbl;
	uint16_t effect_mono_tbl_size;
	struct mt9m114_i2c_reg_conf const *effect_sepia_tbl;
	uint16_t effect_sepia_tbl_size;
	struct mt9m114_i2c_reg_conf const *effect_aqua_tbl;
	uint16_t effect_aqua_tbl_size;
	struct mt9m114_i2c_reg_conf const *effect_negative_tbl;
	uint16_t effect_negative_tbl_size;
	struct mt9m114_i2c_reg_conf const *effect_solarization_tbl;
	uint16_t effect_solarization_tbl_size;
	// White balance
	struct mt9m114_i2c_reg_conf const *wb_default_tbl;
	uint16_t wb_default_tbl_size;
	struct mt9m114_i2c_reg_conf const *wb_sunny_tbl;
	uint16_t wb_sunny_tbl_size;
	struct mt9m114_i2c_reg_conf const *wb_cloudy_tbl;
	uint16_t wb_cloudy_tbl_size;
	struct mt9m114_i2c_reg_conf const *wb_fluorescent_tbl;
	uint16_t wb_fluorescent_tbl_size;
	struct mt9m114_i2c_reg_conf const *wb_incandescent_tbl;
	uint16_t wb_incandescent_tbl_size;
	// ISO
	struct mt9m114_i2c_reg_conf const *iso_default_tbl;
	uint16_t iso_default_tbl_size;
	struct mt9m114_i2c_reg_conf const *iso_100_tbl;
	uint16_t iso_100_tbl_size;
	struct mt9m114_i2c_reg_conf const *iso_200_tbl;
	uint16_t iso_200_tbl_size;
	struct mt9m114_i2c_reg_conf const *iso_400_tbl;
	uint16_t iso_400_tbl_size;
	// Change-config
	struct mt9m114_i2c_reg_conf const *change_config_tbl;
	uint16_t change_config_tbl_size;

	// Preview Size
	struct mt9m114_i2c_reg_conf const *preview_size_vga_tbl;
	uint16_t preview_size_vga_tbl_size;
	struct mt9m114_i2c_reg_conf const *preview_size_full_tbl;
	uint16_t preview_size_full_tbl_size;

};

enum mt9m114_wb_type{
	CAMERA_WB_MIN_MINUS_1,
	CAMERA_WB_AUTO,  /* This list must match aeecamera.h */
	CAMERA_WB_CUSTOM,
	CAMERA_WB_INCANDESCENT,
	CAMERA_WB_FLUORESCENT,
	CAMERA_WB_DAYLIGHT,
	CAMERA_WB_CLOUDY_DAYLIGHT,
	CAMERA_WB_TWILIGHT,
	CAMERA_WB_SHADE,
	CAMERA_WB_MAX_PLUS_1
};

/* Enum Type for different ISO Mode supported */
enum mt9m114_iso_value {
	CAMERA_ISO_AUTO,
	CAMERA_ISO_100,
	CAMERA_ISO_200,
	CAMERA_ISO_400,
	CAMERA_ISO_MAX
};

enum mt9m114_antibanding_type {
	CAMERA_ANTIBANDING_OFF = 0,
	CAMERA_ANTIBANDING_60HZ = 1,
	CAMERA_ANTIBANDING_50HZ = 2,
	CAMERA_MAX_ANTIBANDING,
};

enum mt9m114_resolution_t {
	QTR_SIZE,
	FULL_SIZE,
	INVALID_SIZE
};

enum mt9m114_setting {
	RES_PREVIEW,
	RES_CAPTURE
};

enum mt9m114_reg_update {
	/* Sensor egisters that need to be updated during initialization */
	REG_INIT,
	/* Sensor egisters that needs periodic I2C writes */
	UPDATE_PERIODIC,
	/* All the sensor Registers will be updated */
	UPDATE_ALL,
	/* Not valid update */
	UPDATE_INVALID
};

/*
enum mt9m114_reg_pll {
	E013_VT_PIX_CLK_DIV,
	E013_VT_SYS_CLK_DIV,
	E013_PRE_PLL_CLK_DIV,
	E013_PLL_MULTIPLIER,
	E013_OP_PIX_CLK_DIV,
	E013_OP_SYS_CLK_DIV
};

enum mt9m114_reg_mode {
	E013_X_ADDR_START,
	E013_X_ADDR_END,
	E013_Y_ADDR_START,
	E013_Y_ADDR_END,
	E013_X_OUTPUT_SIZE,
	E013_Y_OUTPUT_SIZE,
	E013_DATAPATH_SELECT,
	E013_READ_MODE,
	E013_ANALOG_CONTROL5,
	E013_DAC_LD_4_5,
	E013_SCALING_MODE,
	E013_SCALE_M,
	E013_LINE_LENGTH_PCK,
	E013_FRAME_LENGTH_LINES,
	E013_COARSE_INTEGRATION_TIME,
	E013_FINE_INTEGRATION_TIME,
	E013_FINE_CORRECTION
};
*/

#endif /* MT9M114_H */
