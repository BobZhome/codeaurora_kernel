/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
*/



#ifndef __CAMSENSOR_DRV__
#define __CAMSENSOR_DRV__

#include <mach/camera.h>

enum camsensor_drv_width_t {
    BYTE_LEN,
    WORD_LEN,
    ADWORD_DATBYTE_LEN,
    ADWORD_DATLONG_LEN,
};

struct camsensor_drv_i2c_reg_conf {
    unsigned short waddr;
    unsigned long wdata;
    enum camsensor_drv_width_t  width;
    unsigned short mdelay_time;
};

struct camsensor_drv_reg_t {

    struct camsensor_drv_i2c_reg_conf const *init_reg_settings;
    uint16_t init_reg_settings_size;

    struct camsensor_drv_i2c_reg_conf const *preview_reg_settings;
    uint16_t preview_reg_settings_size;

    struct camsensor_drv_i2c_reg_conf const *still_reg_settings;
    uint16_t still_reg_settings_size;

    struct camsensor_drv_i2c_reg_conf const *set_wb_auto_reg;
    struct camsensor_drv_i2c_reg_conf const *set_wb_daylight_reg;
    struct camsensor_drv_i2c_reg_conf const *set_wb_cloudy_reg;
    struct camsensor_drv_i2c_reg_conf const *set_wb_incandescent_reg;
    struct camsensor_drv_i2c_reg_conf const *set_wb_fluorescent_reg;
    uint16_t set_wb_reg_size;

    struct camsensor_drv_i2c_reg_conf const *set_effect_none_reg;
    struct camsensor_drv_i2c_reg_conf const *set_effect_mono_reg;
    struct camsensor_drv_i2c_reg_conf const *set_effect_sepia_reg;
    struct camsensor_drv_i2c_reg_conf const *set_effect_negative_reg;
    struct camsensor_drv_i2c_reg_conf const *set_effect_aqua_reg;
    uint16_t set_effect_reg_size;

    struct camsensor_drv_i2c_reg_conf const *set_ae_mode_average_reg;
    struct camsensor_drv_i2c_reg_conf const *set_ae_mode_center_reg;
    struct camsensor_drv_i2c_reg_conf const *set_ae_mode_spot_reg;
    uint16_t set_ae_mode_reg_size;

    struct camsensor_drv_i2c_reg_conf const *set_scene_auto_reg;
    uint16_t set_scene_auto_reg_size;
    struct camsensor_drv_i2c_reg_conf const *set_scene_portrait_reg;
    uint16_t set_scene_portrait_reg_size;
    struct camsensor_drv_i2c_reg_conf const *set_scene_scenery_reg;
    uint16_t set_scene_scenery_reg_size;
    struct camsensor_drv_i2c_reg_conf const *set_scene_night_portrait_reg;
    uint16_t set_scene_night_portrait_reg_size;
    struct camsensor_drv_i2c_reg_conf const *set_scene_night_scenery_reg;
    uint16_t set_scene_night_scenery_reg_size;
    struct camsensor_drv_i2c_reg_conf const *set_scene_action_reg;
    uint16_t set_scene_action_reg_size;

#if defined( FEATURE_KYOCERA_DLCHK_GRADATION_PATTERN ) || defined( FEATURE_KYOCERA_DLCHK_UVFIX_PATTERN )
    struct camsensor_drv_i2c_reg_conf const *dataline_check_mode;
    uint16_t dataline_check_mode_reg_settings_size;

#endif
};

extern struct camsensor_drv_reg_t camsensor_drv_regs;

extern int camsensor_drv_set_flash_mode( unsigned int flash_mode );

#endif
