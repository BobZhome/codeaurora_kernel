/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
*/



#include <linux/i2c.h>
#include <linux/slab.h>
#include <media/msm_camera.h>
#include <mach/hs_io_ctl_a.h>
#include <linux/vmalloc.h>
#include <mach/kcj_dev_info.h>
#include "camsensor_drv.h"

#include <linux/leds-pmic8058.h>

#include <linux/kthread.h>

#define SEQ_LOG_SWITCH
#undef SEQ_LOG_SWITCH
#ifdef SEQ_LOG_SWITCH
#define SEQ_LOG(fmt, args...) printk(KERN_INFO fmt, ##args)
#else
#define SEQ_LOG(fmt, args...) do{ } while(0)
#endif

#define ERR_LOG_SWITCH

#ifdef ERR_LOG_SWITCH
#define ERR_LOG(fmt, args...) printk(KERN_ERR fmt, ##args)
#else
#define ERR_LOG(fmt, args...) do{ } while(0)
#endif

#define API_LOG_SWITCH
#undef API_LOG_SWITCH
#ifdef API_LOG_SWITCH
#define API_LOG(fmt, args...) printk(KERN_INFO fmt, ##args)
#else
#define API_LOG(fmt, args...) do{ } while(0)
#endif

#define DBG_LOG_SWITCH
#undef DBG_LOG_SWITCH
#ifdef DBG_LOG_SWITCH
#define DBG_LOG(fmt, args...) printk(KERN_NOTICE fmt, ##args)
#else
#define DBG_LOG(fmt, args...) do{ } while(0)
#endif

#define LED_LOG_SWITCH
#undef LED_LOG_SWITCH
#ifdef LED_LOG_SWITCH
#define LED_LOG(fmt, args...) printk(KERN_INFO fmt, ##args)
#else
#define LED_LOG(fmt, args...) do{ } while(0)
#endif

#define CAMSENSOR_DEFAULT_CLOCK_RATE  24000000
#define TRUE 1
#define FALSE 0

#define EXIF_F_NUMBER           ((uint16_t)(280))
#define EXIF_FOCAL_LENGTH       ((uint16_t)(270))
#define EXIF_FOCAL_LENGTH_35MM  ((uint16_t)(3340))
#define EXIF_FLASH_ON           ((uint8_t)(0x09))
#define EXIF_FLASH_OFF          ((uint8_t)(0x10))
#define EXIF_FLASH_AUTO_OFF     ((uint8_t)(0x18))
#define EXIF_FLASH_AUTO_ON      ((uint8_t)(0x19))

#define MAX_GAIN_VALUE 6225

#define MAX_EXPTIME_NIGTH_VALUE  0x00694920
#define MAX_EXPTIME_NORMAL_VALUE 0x00325AA0

#define CAMSENSOR_VSYNC_GPIO            14
#define CAM_DRV_CURRENT_THREAD_WAIT      0
#define CAM_DRV_THREAD_WAKE_UP           1
#define CAMERA_INT_VSYNC_TIMEOUT      1000
#define CAMERA_INT_KTHREAD_TIMEOUT    1000

#define KTHREAD_PROC_STILL_FRAME      0x00000001

#define FPS_30 30
#define FPS_15 15
#define FPS_10 10
#define FPS_7   7
#define FPS_5   5

struct camsensor_drv_ctrl {
    const struct msm_camera_sensor_info *sensordata;
    struct camsensor_drv_reg_t *camsensor_drv_regs;
    enum msm_s_resolution prev_res;
    int sensormode;
    int flash_on_flg;
    int flash_up_flg;

    struct task_struct *camdev_th;
    unsigned int api_after_proc_kind;

    uint8_t  again;
    uint8_t  wb_mode;
    uint8_t  effect_mode;
    uint8_t  aec_mode;
    uint8_t  scene_mode;
    uint8_t  flash_mode;
    uint16_t iso_speed;
    uint8_t  flash_exif_data;
    unsigned long exposure_time;
};

struct camsensor_drv_work {
    struct work_struct work;
};

typedef enum
{
    CAMERA_DRV_VIDEO_LIGHT,
    CAMERA_DRV_MAIN_FLASH,
    CAMERA_DRV_FLASH_OFF,
    CAMERA_DRV_FLASH_CTRL_MAX,
} camsensor_drv_flash_type;

struct camsensor_drv_vsync_irq_type {

    spinlock_t camsensor_drv_spin_lock;

    wait_queue_head_t camsensor_drv_wait_queue;
    atomic_t v_frame_count;

    wait_queue_head_t cam_kthread_wait_queue;
    atomic_t kthread_frame_count;
    int kthread_frame_cnt;
};

typedef enum
{
    VSYNC_IRQ_LOW,
    VSYNC_IRQ_HIGH,
    VSYNC_IRQ_MAX,
} camsensor_drv_vsync_attribute_type;

struct camsensor_drv_led_setting_st {
    unsigned short aectl;
    unsigned short awbctl;
    unsigned short awbctl3;
    unsigned long  exptime;
    unsigned short again;
    unsigned short traceCntTh;
    unsigned short traceStepLmt;
    unsigned short ylvl_bofore;
    unsigned short ymean_bofore;
    unsigned short ylvl;
    unsigned short ylvl_c414;
    unsigned short ymean;
    unsigned short hz;
    unsigned long  anti_band;
    unsigned short awb_rgain;
    unsigned short awb_bgain;
    unsigned long  new_exptime;
    unsigned short new_again;
};

typedef enum
{
    LED_SETTING_JUDGE_AUTOFLASH,
    LED_SETTING_BEFORE_PREFLASH,
    LED_SETTING_CALCULATE_VALUE,
    LED_SETTING_RETURN_REGISTER,
    LED_SETTING_MAX,
} camsensor_drv_led_setting_fase;

typedef struct{
  unsigned short  min;
  unsigned short  max;
  unsigned short  value;
} camsensor_iso_struct;

static struct camsensor_drv_ctrl *camsensor_drv_ctrl;
static struct i2c_client    *camsensor_drv_client;
static struct camsensor_drv_work *camsensor_drv_sensorw;
static struct i2c_device_id camsensor_drv_i2c_id[] = {
    { MCAM_I2C_NAME, 0},
    { },
};

struct camsensor_drv_i2c_reg_conf camsensor_drv_power_on_reg[] = {
{ 0x03, 0x00, BYTE_LEN, 0},
{ 0x01, 0x51, BYTE_LEN, 0},
{ 0x01, 0x53, BYTE_LEN, 0},
{ 0x01, 0x51, BYTE_LEN, 0},
};

struct camsensor_drv_i2c_reg_conf camsensor_drv_power_off_reg[] = {
{ 0x03, 0x02, BYTE_LEN, 0},
{ 0x55, 0x10, BYTE_LEN, 0},
};

static uint32_t camsensor_i2c_gpio_IN_NP[] = {
  GPIO_CFG(GPIO_0, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
  GPIO_CFG(GPIO_1, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static uint32_t camsensor_i2c_gpio_IN_PD[] = {
  GPIO_CFG(GPIO_0, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
  GPIO_CFG(GPIO_1, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
};

static uint32_t camsensor_mclk_gpio_OUT_NP[] = {
  GPIO_CFG(15, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static uint32_t camsensor_mclk_gpio_OUT_PD[] = {
  GPIO_CFG(15, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
};

static struct camsensor_drv_vsync_irq_type camsensor_drv_vsync_irq;
static atomic_t start_vsync_count;
static atomic_t start_kthread_vsync_count;

static bool camsensor_kthread_stop_flg = false;

static camsensor_iso_struct camsensor_iso_conversion_tbl[] =
{

  {     0,  1500,              80 },
  {  1500,  2025,             100 },
  {  2025,  2550,             125 },
  {  2550,  3075,             160 },
  {  3075,  3600,             200 },
  {  3600,  4125,             250 },
  {  4125,  4650,             320 },
  {  4650,  5175,             400 },
  {  5175,  5700,             500 },
  {  5700,  MAX_GAIN_VALUE,   640 },
};

static camera_scene_info_type scene_info[ CAM_SCENE_MAX ] =
{

  {
    CAM_WB_AUTO,
    CAM_AF_MODE_NORMAL,
    CAM_AEC_CENTER,
    CAM_ISO_AUTO,
    CAM_FLASH_OFF,
    0,
    CAM_ANTIBANDING_AUTO,
    0,
    FPS_30,
    0,
    0,
    0,
    CAM_EFFECT_NONE,
    TRUE
  },

  {
    CAM_WB_AUTO,
    CAM_AF_MODE_NORMAL,
    CAM_AEC_CENTER,
    CAM_ISO_AUTO,
    CAM_FLASH_AUTO,
    0,
    CAM_ANTIBANDING_AUTO,
    0,
    FPS_30,
    0,
    0,
    0,
    CAM_EFFECT_NONE,
    TRUE
  },

  {
    CAM_WB_AUTO,
    CAM_AF_MODE_NORMAL,
    CAM_AEC_AVERAGE,
    CAM_ISO_AUTO,
    CAM_FLASH_AUTO,
    0,
    CAM_ANTIBANDING_AUTO,
    0,
    FPS_30,
    0,
    0,
    0,
    CAM_EFFECT_NONE,
    TRUE
  },

  {
    CAM_WB_AUTO,
    CAM_AF_MODE_NORMAL,
    CAM_AEC_CENTER,
    CAM_ISO_AUTO,
    CAM_FLASH_ON,
    0,
    CAM_ANTIBANDING_AUTO,
    0,
    FPS_30,
    0,
    0,
    0,
    CAM_EFFECT_NONE,
    TRUE
  },

  {
    CAM_WB_AUTO,
    CAM_AF_MODE_NORMAL,
    CAM_AEC_AVERAGE,
    CAM_ISO_AUTO,
    CAM_FLASH_OFF,
    0,
    CAM_ANTIBANDING_AUTO,
    0,
    FPS_30,
    0,
    0,
    0,
    CAM_EFFECT_NONE,
    TRUE
  },

  {
    CAM_WB_AUTO,
    CAM_AF_MODE_NORMAL,
    CAM_AEC_CENTER,
    CAM_ISO_400,
    CAM_FLASH_OFF,
    0,
    CAM_ANTIBANDING_AUTO,
    0,
    FPS_30,
    0,
    0,
    0,
    CAM_EFFECT_NONE,
    TRUE
  },
};

static DECLARE_WAIT_QUEUE_HEAD(camsensor_drv_wait_queue);
DEFINE_MUTEX(camsensor_drv_mutex);

static int L_camsensor_drv_probe( struct platform_device *pdev );
static int L_camsensor_drv_probe_init_sensor( const struct msm_camera_sensor_info *data );
static int L_camsensor_drv_sensor_config( void __user *argp );
static int L_camsensor_drv_sensor_init( const struct msm_camera_sensor_info *data );
static int L_camsensor_drv_sensor_probe( const struct msm_camera_sensor_info *info, struct msm_sensor_ctrl *sensor );
static int L_camsensor_drv_sensor_release( void );
static int L_camsensor_drv_pwd_config( unsigned on_off );
static int32_t L_camsensor_drv_power_down( void );
static int L_camsensor_drv_set_reg_table( void );
static int32_t L_camsensor_drv_set_parm_reg_table( struct camsensor_drv_i2c_reg_conf const *reg_cfg_tbl, int num );
static int32_t L_camsensor_drv_setting( enum msm_s_reg_update rupdate,enum msm_s_setting rsetting );
static int32_t L_camsensor_drv_set_gpio(uint32_t *table, int len);
static int L_camsensor_drv_set_wb( camera_wb_type wb );
static int L_camsensor_drv_set_effect( camera_effect_type effect );
static int L_camsensor_drv_set_ae_mode( camera_auto_exposure_type aec_mode );
static int L_camsensor_drv_set_scene( camera_scene_type scene_mode, int write_mode );
static int L_camsensor_drv_get_sensor_exif_info( void );

static int L_camsensor_drv_i2c_probe( struct i2c_client *client, const struct i2c_device_id *id );
static int32_t L_camsensor_drv_i2c_write_table( struct camsensor_drv_i2c_reg_conf const *reg_cfg_tbl, int num );
static int L_camsensor_drv_i2c_init_client( struct i2c_client *client );
static int32_t L_camsensor_drv_i2c_write( unsigned short saddr, unsigned short waddr, unsigned long wdata, enum camsensor_drv_width_t width);
static int32_t L_camsensor_drv_i2c_txdata( unsigned short saddr,unsigned char *txdata,int length );

static int32_t L_camsensor_drv_i2c_read( unsigned short saddr, unsigned short raddr, unsigned short *rdata, enum camsensor_drv_width_t width );
static int L_camsensor_drv_i2c_rxdata( unsigned short saddr, unsigned char *rxdata, int length );

static int L_camsensor_drv_read_hw_ver( void );

static int L_camsensor_drv_pmic_flash_ctrl( camsensor_drv_flash_type flash_mode );

#ifdef FEATURE_KYOCERA_DLCHK_GRADATION_PATTERN
static int32_t L_camsensor_drv_dataline_check_config(int mode);
#endif
#ifdef FEATURE_KYOCERA_DLCHK_UVFIX_PATTERN
static int32_t L_camsensor_drv_set_dataline_check_mode( common_dl_check_t* dl_check);
#endif

static int L_camsensor_drv_wait_vsync( int frame );
static irqreturn_t L_camsensor_drv_vsync_interrupt( int irq, void *handle );
static int L_camsensor_drv_create_irq( void );
static int L_camsensor_drv_change_irq( camsensor_drv_vsync_attribute_type irq_type );
static void L_camsensor_drv_vsync_proc_entry( unsigned int entry_proc );

static int L_camsensor_drv_kthread_run( int frame );
static int L_camsensor_drv_kthread_proc( void* data );

static int32_t L_camsensor_drv_prepare_snapshot( void );
static int L_camsensor_drv_get_exif_info( common_exif_info_t *exif );
static int L_camsensor_drv_get_scene_info( camera_scene_info_type* info );

static int32_t L_camsensor_drv_set_led_setting( camsensor_drv_led_setting_fase fase );
static int32_t L_camsensor_drv_set_led_setting_calc( unsigned long exptime, unsigned short again, unsigned short ylvl, unsigned short ymean, unsigned long anti, unsigned long*  new_exptime, unsigned short* new_again );

static int L_camsensor_drv_i2c_probe
(
  struct i2c_client *client,
  const struct i2c_device_id *id
)
{
    int rc = 0;

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        ERR_LOG("%s i2c_check_functionality ERR\n",__func__);
        kfree(camsensor_drv_sensorw);
        camsensor_drv_sensorw = NULL;
        return rc;
    }

    camsensor_drv_sensorw = kzalloc(sizeof(struct camsensor_drv_work), GFP_KERNEL);
    if (!camsensor_drv_sensorw)
    {
        ERR_LOG("%s failed!\n",__func__);
        rc = -ENOMEM;
        kfree(camsensor_drv_sensorw);
        camsensor_drv_sensorw = NULL;
        return rc;
    }

    i2c_set_clientdata(client, camsensor_drv_sensorw);
    L_camsensor_drv_i2c_init_client(client);
    camsensor_drv_client = client;

    return rc;
}

static int L_camsensor_drv_i2c_init_client
(
  struct i2c_client *client
)
{

    init_waitqueue_head(&camsensor_drv_wait_queue);

    return 0;
}

static int32_t L_camsensor_drv_i2c_read(
    unsigned short saddr,
    unsigned short raddr,
    unsigned short *rdata,
    enum camsensor_drv_width_t width
)
{
    int32_t rc = 0;
    unsigned char buf[4];
    unsigned long read_data = 0;
    int length;

    if (!rdata)
    {
        ERR_LOG("%s param err NULL\n",__func__);
        return -EIO;
    }

    memset(buf, 0, sizeof(buf));

    buf[0] = (raddr & 0x00FF);

    switch (width)
    {
        case WORD_LEN:
            length = 2;
            break;

        case BYTE_LEN:
        case ADWORD_DATBYTE_LEN:
            length = 1;
            break;

        case ADWORD_DATLONG_LEN:
            length = 4;
            break;
        default:
            ERR_LOG("%s Param Err [%d]\n",__func__,rc);
            rc = -EIO;
            break;
    }

    if (!(rc < 0))
    {
        rc = L_camsensor_drv_i2c_rxdata(saddr, buf, length);

        if (rc < 0)
        {
            ERR_LOG("%s i2c_rxdata err [%d]\n",__func__,rc);
            return rc;
        }

        switch (width)
        {
            case WORD_LEN:
                *rdata = buf[0] << 8 | buf[1];
                break;
            case BYTE_LEN:
            case ADWORD_DATBYTE_LEN:
                *rdata = buf[0];
                break;
            case ADWORD_DATLONG_LEN:
                read_data = (( buf[0] << 24) | ( buf[1] << 16 ) | ( buf[2] << 8 ) | ( buf[3] )) ;
                break;
            default:
                ERR_LOG("%s Param Err [%d]\n",__func__,rc);
                rc = -EIO;
                break;
        }
    }

    return rc;
}

static int L_camsensor_drv_i2c_rxdata
(
  unsigned short saddr,
  unsigned char *rxdata,
  int length
)
{
    struct i2c_msg msgs[] = {
        {
          saddr, 0, length, rxdata,
        },
        {
          saddr, I2C_M_RD, length, rxdata,
        },
    };
    int ret;

    ret = i2c_transfer(camsensor_drv_client->adapter, msgs, 2);
    if (ret < 0)
    {
        ERR_LOG("%s failed!  ret:[%d]\n",__func__ ,ret);
        return -EIO;
    }

    return 0;
}

static int32_t L_camsensor_drv_i2c_write_table
(
  struct camsensor_drv_i2c_reg_conf const *reg_cfg_tbl,
  int num
)
{
    int i;
    int cnt = 0;
    unsigned char *buf;
    int32_t rc = -EIO;

    buf = vmalloc(num);
    if(buf==NULL)
    {
        ERR_LOG("vmalloc err size = %d\n",num);
        return rc;
    }

    for (i = 0; i < num; i++)
    {
        if( (reg_cfg_tbl->waddr == ((reg_cfg_tbl+1)->waddr - 1)) && (reg_cfg_tbl->mdelay_time == 0) )
        {
            if(cnt == 0)
            {

                buf[cnt]   = (unsigned char)(reg_cfg_tbl->waddr & 0x00FF);
                buf[cnt+1] = (unsigned char)(reg_cfg_tbl->wdata & 0x000000FF);

                cnt = 2;
            }
            else
            {

                buf[cnt] = (unsigned char)(reg_cfg_tbl->wdata & 0x000000FF);

                cnt++;
            }
        }
        else
        {
            if(cnt > 0)
            {

                buf[cnt] = (unsigned char)(reg_cfg_tbl->wdata & 0x000000FF);

                cnt++;

                L_camsensor_drv_i2c_txdata(camsensor_drv_client->addr, buf, cnt);
                cnt = 0;
            }
            else
            {

                rc = L_camsensor_drv_i2c_write(camsensor_drv_client->addr,reg_cfg_tbl->waddr, reg_cfg_tbl->wdata, reg_cfg_tbl->width);
            }

            if (rc < 0)
            {
                ERR_LOG("L_camsensor_drv_i2c_write ERR = [%d] \n",rc);
                break;
            }

            if (reg_cfg_tbl->mdelay_time != 0)
            {

                mdelay(reg_cfg_tbl->mdelay_time);
            }
        }
        reg_cfg_tbl++;
    }

    vfree(buf);

    return rc;
}

static int32_t L_camsensor_drv_i2c_write
(
  unsigned short saddr,
  unsigned short waddr,
  unsigned long  wdata,
  enum camsensor_drv_width_t  width
)
{
    int32_t rc = 0;
    unsigned char buf[6];
    int length;

    memset(buf, 0, sizeof(buf));
    switch (width)
    {
        case WORD_LEN:
            buf[0] = (unsigned char)((waddr & 0xFF00)>>8);
            buf[1] = (unsigned char)((waddr & 0x00FF));
            buf[2] = (unsigned char)((wdata & 0xFF00) >> 8);
            buf[3] = (unsigned char)((wdata & 0x00FF));
            length = 4;
            break;
        case BYTE_LEN:
            buf[0] = (unsigned char)(waddr & 0x000000FF);
            buf[1] = (unsigned char)(wdata & 0x000000FF);
            length = 2;
            break;
        case ADWORD_DATBYTE_LEN:
            buf[0] = (unsigned char)((waddr & 0xFF00)>>8);
            buf[1] = (unsigned char)(waddr & 0x00FF);
            buf[2] = (unsigned char)(wdata & 0x000000FF);
            length = 3;
            break;
        case ADWORD_DATLONG_LEN:
            buf[0] = (unsigned char)((waddr & 0xFF00)>>8);
            buf[1] = (unsigned char)(waddr & 0x00FF);
            buf[2] = (unsigned char)((wdata & 0x000000FF));
            buf[3] = (unsigned char)((wdata & 0x0000FF00)>>8);
            buf[4] = (unsigned char)((wdata & 0x00FF0000)>>16);
            buf[5] = (unsigned char)((wdata & 0xFF000000)>>24);
            length = 6;
            break;
        default:
            rc = -EIO;
            ERR_LOG("%s Param Err\n", __func__);
            break;
    }

    if (!(rc < 0))
    {
        rc = L_camsensor_drv_i2c_txdata(saddr, buf, length);
        if (rc < 0)
        {
            ERR_LOG("i2c_write_w failed, addr = 0x%x, val = 0x%x!\n",waddr, (unsigned int)wdata);
        }
    }

    return rc;
}

static int32_t L_camsensor_drv_i2c_txdata
(
  unsigned short saddr,
  unsigned char *txdata,
  int length
)
{
    int ret;
    struct i2c_msg msg[] = {
        {
          saddr, 0, length, txdata,
        },
    };

    ret = i2c_transfer(camsensor_drv_client->adapter, msg, 1);
    if (ret < 0)
    {
        ERR_LOG("L_camsensor_drv_i2c_txdata failed! ret:[%d]\n",ret);
        return -EIO;
    }

    return 0;
}

static int __exit L_camsensor_drv_i2c_remove
(
  struct i2c_client *client
)
{
    return 0;
}

static struct i2c_driver camsensor_drv_i2c_driver = {
    .id_table = camsensor_drv_i2c_id,
    .probe  = L_camsensor_drv_i2c_probe,
    .remove = __exit_p(L_camsensor_drv_i2c_remove),
    .driver = {
        .name = MCAM_I2C_NAME,
    },
};

static int L_camsensor_drv_create_irq( void )
{
    int rc = 0;

    init_waitqueue_head( &camsensor_drv_vsync_irq.camsensor_drv_wait_queue);

    rc = request_irq( gpio_to_irq(CAMSENSOR_VSYNC_GPIO),
                      L_camsensor_drv_vsync_interrupt, IRQF_TRIGGER_RISING, "camsensor_drv_vsync", 0);

    return rc;
}

static int L_camsensor_drv_change_irq( camsensor_drv_vsync_attribute_type irq_type )
{
    int rc = 0;

    free_irq(gpio_to_irq(CAMSENSOR_VSYNC_GPIO),0);

    if( irq_type == VSYNC_IRQ_LOW )
    {

        rc = request_irq( gpio_to_irq(CAMSENSOR_VSYNC_GPIO),
                          L_camsensor_drv_vsync_interrupt, IRQF_TRIGGER_FALLING, "camsensor_drv_vsync", 0);
        if( rc < 0 )
        {
            ERR_LOG("VSYNC_IRQ_LOW request_irq Err:%d\n", rc );
        }
    }
    else if( irq_type == VSYNC_IRQ_HIGH )
    {

        rc = request_irq( gpio_to_irq(CAMSENSOR_VSYNC_GPIO),
                          L_camsensor_drv_vsync_interrupt, IRQF_TRIGGER_RISING, "camsensor_drv_vsync", 0);
        if( rc < 0 )
        {
            ERR_LOG("VSYNC_IRQ_HIGH request_irq Err:%d\n", rc );
        }
    }
    else
    {
        ERR_LOG("irq_type ILLEGAL VALUE:%d\n", irq_type );
    }

    return rc;
}

static int L_camsensor_drv_kthread_run( int frame )
{
    int rc = 0;

    init_waitqueue_head( &camsensor_drv_vsync_irq.cam_kthread_wait_queue );

    camsensor_drv_vsync_irq.kthread_frame_cnt = frame;

    camsensor_kthread_stop_flg = false;

    camsensor_drv_ctrl->camdev_th = kthread_run( L_camsensor_drv_kthread_proc, &camsensor_drv_vsync_irq.kthread_frame_cnt, "camdev_thread" );

    if(IS_ERR( camsensor_drv_ctrl->camdev_th ))
    {
        ERR_LOG("kthread create Failed\n");
        rc = PTR_ERR( camsensor_drv_ctrl->camdev_th );
        camsensor_drv_ctrl->camdev_th = NULL;
    }

    return rc;

}

static int L_camsensor_drv_kthread_proc( void* data )
{
    int rc = 0;
    int frame;

    mutex_lock(&camsensor_drv_mutex);
    if(!camsensor_kthread_stop_flg)
    {
        frame = *(int*)data;

        atomic_set( &start_kthread_vsync_count, true );

        rc = wait_event_interruptible_timeout( camsensor_drv_vsync_irq.cam_kthread_wait_queue,
                                               atomic_read( &camsensor_drv_vsync_irq.kthread_frame_count ) == frame,
                                               msecs_to_jiffies(CAMERA_INT_KTHREAD_TIMEOUT)
                                             );
        if( rc <= 0 )
        {
            ERR_LOG("%s Vsync Wait Err:%d\n", __func__, rc );
        }

        atomic_set( &camsensor_drv_vsync_irq.kthread_frame_count, 0 );

        atomic_set( &start_kthread_vsync_count, false );

        if( camsensor_drv_ctrl->api_after_proc_kind == KTHREAD_PROC_STILL_FRAME )
        {

            rc |= L_camsensor_drv_change_irq( VSYNC_IRQ_HIGH );

            rc |= L_camsensor_drv_wait_vsync(1);

            rc |= L_camsensor_drv_get_sensor_exif_info();
            if( rc < 0 )
            {
                ERR_LOG("KTHREAD GET SENSOR EXIF DATA Err\n");
            }

            if( camsensor_drv_ctrl->flash_on_flg == 1 )
            {

                rc |= L_camsensor_drv_change_irq( VSYNC_IRQ_LOW );

                rc |= L_camsensor_drv_wait_vsync(1);

                rc |= L_camsensor_drv_pmic_flash_ctrl( CAMERA_DRV_FLASH_OFF );
                if( rc < 0 )
                {
                    ERR_LOG("KTHREAD FLASH OFF Err\n");
                }
            }
        }

        camsensor_drv_ctrl->api_after_proc_kind = 0x00000000;
    }
    else
    {
        ERR_LOG("KTHREAD STOP\n");
    }

    mutex_unlock(&camsensor_drv_mutex);

    return rc;

}

static void L_camsensor_drv_vsync_proc_entry( unsigned int entry_proc )
{

    camsensor_drv_ctrl->api_after_proc_kind |= entry_proc;

}

static irqreturn_t L_camsensor_drv_vsync_interrupt
(
  int irq,
  void *handle
)
{
    unsigned long flags;

    spin_lock_irqsave( &camsensor_drv_vsync_irq.camsensor_drv_spin_lock, flags );

    if( atomic_read( &start_vsync_count ) )
    {
        printk("Vsync Count Start\n");

        atomic_add( 1, &camsensor_drv_vsync_irq.v_frame_count );

        wake_up_interruptible( &camsensor_drv_vsync_irq.camsensor_drv_wait_queue );
    }

    if( atomic_read( &start_kthread_vsync_count ) )
    {
        printk("kthread Vsync Count Start\n");

        atomic_add( 1, &camsensor_drv_vsync_irq.kthread_frame_count );

        wake_up_interruptible( &camsensor_drv_vsync_irq.cam_kthread_wait_queue );
    }
    spin_unlock_irqrestore( &camsensor_drv_vsync_irq.camsensor_drv_spin_lock, flags );

    return IRQ_HANDLED;
}

static int L_camsensor_drv_wait_vsync( int frame )
{
    int rc = 0;

    atomic_set( &start_vsync_count, true );

    rc = wait_event_interruptible_timeout( camsensor_drv_vsync_irq.camsensor_drv_wait_queue,
                                      atomic_read( &camsensor_drv_vsync_irq.v_frame_count ) == frame,
                                      msecs_to_jiffies( CAMERA_INT_VSYNC_TIMEOUT ));

    if( rc == 0 )
    {
        ERR_LOG("Vsync Wait Time Out\n");
        rc = -ETIMEDOUT;
    }

    atomic_set( &camsensor_drv_vsync_irq.v_frame_count, 0 );

    atomic_set( &start_vsync_count, false );

    return rc;
}

int camsensor_drv_set_flash_mode( unsigned int flash_mode )
{
    int rc = 0;

    switch( flash_mode )
    {
        case MSM_CAMERA_LED_FLASH_OFF:
          camsensor_drv_ctrl->flash_mode = CAM_FLASH_OFF;
          break;
        case MSM_CAMERA_LED_FLASH_AUTO:
          camsensor_drv_ctrl->flash_mode = CAM_FLASH_AUTO;
          break;
        case MSM_CAMERA_LED_FLASH_ON:
          camsensor_drv_ctrl->flash_mode = CAM_FLASH_ON;
          break;
        case MSM_CAMERA_LED_FLASH_TORCH:
          camsensor_drv_ctrl->flash_mode = CAM_FLASH_TORCH;
          break;
        default:
          camsensor_drv_ctrl->flash_mode = CAM_FLASH_OFF;
          break;
    }

    if( camsensor_drv_ctrl->flash_mode == CAM_FLASH_TORCH )
    {
        rc = L_camsensor_drv_pmic_flash_ctrl( CAMERA_DRV_VIDEO_LIGHT );
        if( rc < 0 )
        {
            ERR_LOG("PMIC Flash Err\n");
        }
    }
    if( camsensor_drv_ctrl->flash_mode == CAM_FLASH_OFF )
    {
        rc = L_camsensor_drv_pmic_flash_ctrl( CAMERA_DRV_FLASH_OFF );
        if( rc < 0 )
        {
            ERR_LOG("PMIC Flash Err\n");
        }
    }

    return rc;
}

static int L_camsensor_drv_pwd_config
(
  unsigned on_off
)
{
    int32_t rc = 0;
    unsigned long flags;

    local_irq_save(flags);
    local_irq_disable();

    if(on_off)
    {

        gpio_direction_output( GPIO_CAM_SCL, GPIO_LO );
        gpio_direction_output( GPIO_CAM_SDA, GPIO_LO );

        HS_A_CAM_VDD_IO_CNT_HI();
        udelay(100);

        HS_A_CAM_VDD_A_CNT_HI();
        udelay(300);

        HS_A_CAM_VDD_D_CNT_HI();
        mdelay(3);

        msm_camio_clk_enable(CAMIO_CAM_MCLK_CLK);
        msm_camio_clk_rate_set(CAMSENSOR_DEFAULT_CLOCK_RATE);
        L_camsensor_drv_set_gpio(camsensor_mclk_gpio_OUT_NP,ARRAY_SIZE(camsensor_mclk_gpio_OUT_NP));
        mdelay(3);

        HS_A_CAM_PWDN_HI();
        mdelay(40);

        HS_A_CAM_RST_HI();
        udelay(100);

        L_camsensor_drv_set_gpio(camsensor_i2c_gpio_IN_NP,ARRAY_SIZE(camsensor_i2c_gpio_IN_NP));

        gpio_direction_output( GPIO_CAM_SCL, GPIO_HI );
        gpio_direction_output( GPIO_CAM_SDA, GPIO_HI );
        udelay(100);

        rc = L_camsensor_drv_i2c_write_table(&camsensor_drv_power_on_reg[0],ARRAY_SIZE(camsensor_drv_power_on_reg));
        if(rc < 0)
        {
            ERR_LOG("Camera Power ON! failed camsensor_drv_i2c_write! (rc=%d)\n",rc);
        }

        kcj_dev_info_update_cam(DEV_INFO_CAM_PREVIEW);
    }
    else
    {

        rc = L_camsensor_drv_i2c_write_table(&camsensor_drv_power_off_reg[0],ARRAY_SIZE(camsensor_drv_power_off_reg));
        if(rc < 0)
        {
            ERR_LOG("Camera Power OFF! failed camsensor_drv_i2c_write! (rc=%d)\n",rc);
        }

        camsensor_i2c_gpio_IN_PD[0] = GPIO_CFG(GPIO_CAM_SCL, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA);
        camsensor_i2c_gpio_IN_PD[1] = GPIO_CFG(GPIO_CAM_SDA, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA);
        L_camsensor_drv_set_gpio(camsensor_i2c_gpio_IN_PD,ARRAY_SIZE(camsensor_i2c_gpio_IN_PD));

        gpio_direction_output( GPIO_CAM_SCL, GPIO_LO );
        gpio_direction_output( GPIO_CAM_SDA, GPIO_LO );
        udelay(100);

        HS_A_CAM_RST_LO();
        udelay(100);

        msm_camio_clk_disable(CAMIO_CAM_MCLK_CLK);
        L_camsensor_drv_set_gpio(camsensor_mclk_gpio_OUT_PD,ARRAY_SIZE(camsensor_mclk_gpio_OUT_PD));
        udelay(100);

        HS_A_CAM_PWDN_LO();
        udelay(100);

        HS_A_CAM_VDD_D_CNT_LO();
        udelay(200);

        HS_A_CAM_VDD_A_CNT_LO();
        udelay(100);

        HS_A_CAM_VDD_IO_CNT_LO();

        kcj_dev_info_update_cam(DEV_INFO_CAM_OFF);
    }

    local_irq_enable();
    local_irq_restore(flags);

    return rc;
}

static int32_t L_camsensor_drv_power_down
(
  void
)
{
    int32_t rc = 0;

    L_camsensor_drv_pwd_config(0);

    kfree(camsensor_drv_ctrl);
    camsensor_drv_ctrl = NULL;

    return rc;
}

static int32_t L_camsensor_drv_setting
(
  enum msm_s_reg_update rupdate,
  enum msm_s_setting rsetting
)
{
    int32_t rc = 0;
    struct camsensor_drv_i2c_reg_conf const* set_reg;
    uint16_t set_reg_size;

    switch (rupdate)
    {
        case S_UPDATE_PERIODIC:
            if (rsetting == S_RES_PREVIEW)
            {

            }
            else if(rsetting == S_RES_CAPTURE)
            {

            }
            else
            {
                rc = -EINVAL;
            }
            break;
        case S_REG_INIT:

            rc = L_camsensor_drv_create_irq();
            if( rc < 0 )
            {
                ERR_LOG("Create Irq Failed\n");
            }

            if (rsetting == S_RES_PREVIEW || rsetting == S_RES_CAPTURE)
            {
                set_reg = &camsensor_drv_regs.init_reg_settings[0];
                set_reg_size = camsensor_drv_regs.init_reg_settings_size;
            }
            else
            {
                rc = -EINVAL;
            }
            break;
        default:
            rc = -EINVAL;
            break;
    }

    if(rc == 0)
    {
        if(rupdate != S_REG_INIT)
        {

        }
        else
        {
            rc = L_camsensor_drv_i2c_write_table(set_reg, set_reg_size);
        }

        if(rc < 0)
        {
            ERR_LOG("failed camsensor_drv_i2c_write! (rc=%d)\n",rc);
        }
        else
        {

        }
    }

    return rc;
}

static int32_t L_camsensor_drv_set_gpio(uint32_t *table, int len)
{
    int n;
    int32_t rc = 0;
    for (n = 0; n < len; n++) {
        rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
        if (rc) {
            ERR_LOG("%s gpio_tlmm_config ERROR ! len=[%d] table=[%d] rc=[%d]",__func__, len, n, rc);
        }
    }
    return rc;
}

static int L_camsensor_drv_read_hw_ver
(
  void
)
{
    int rc = 0;

    return rc;
}

static int L_camsensor_drv_probe_init_sensor
(
  const struct msm_camera_sensor_info *data
)
{
    int rc = 0;

#if defined(CONFIG_TARGET_PROTOTYPE_WS0)
    if(0)
#else

    if( HS_A_GET_HW_ID() > 0)
#endif

    {

        strcpy(camsensor_drv_i2c_id[0].name, MCAM_I2C_NAME_QUP);
        camsensor_drv_i2c_driver.driver.name = MCAM_I2C_NAME_QUP;
        camsensor_i2c_gpio_IN_NP[0] = GPIO_CFG(GPIO_CAM_SCL, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);
        camsensor_i2c_gpio_IN_NP[1] = GPIO_CFG(GPIO_CAM_SDA, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);
    }
    else
    {

        strcpy(camsensor_drv_i2c_id[0].name, MCAM_I2C_NAME);
        camsensor_drv_i2c_driver.driver.name = MCAM_I2C_NAME;
        camsensor_i2c_gpio_IN_NP[0] = GPIO_CFG(GPIO_CAM_SCL, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);
        camsensor_i2c_gpio_IN_NP[1] = GPIO_CFG(GPIO_CAM_SDA, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);
    }

    return rc;
}

static int L_camsensor_drv_set_reg_table
(
  void
)
{
    int rc = 0;

    camsensor_drv_ctrl->camsensor_drv_regs = &camsensor_drv_regs;

    return rc;
}

static int32_t L_camsensor_drv_set_parm_reg_table
(
  struct camsensor_drv_i2c_reg_conf const *reg_cfg_tbl,
  int num
)
{
    int rc = 0;

    if( reg_cfg_tbl == NULL )
    {
        ERR_LOG("%s Param NULL Err\n",__func__);
        return -EPERM;
    }

    rc = L_camsensor_drv_change_irq( VSYNC_IRQ_HIGH );
    if( rc < 0 )
    {
        ERR_LOG("L_camsensor_drv_set_parm_reg_table L_camsensor_drv_change_irq()Err\n");
        return rc;
    }

    rc = L_camsensor_drv_wait_vsync(1);
    if( rc < 0 )
    {
        ERR_LOG("L_camsensor_drv_set_parm_reg_table 1 Wait Vsync High Err\n");
        return rc;
    }

    rc = L_camsensor_drv_i2c_write_table(reg_cfg_tbl, num);
    if( rc < 0 )
    {
        ERR_LOG("failed camsensor_drv_i2c_write! (rc=%d)\n",rc);
        return rc;
    }

    return rc;
}

static int L_camsensor_drv_get_scene_info
(
  camera_scene_info_type* info
)
{

    if( info == NULL )
    {
        ERR_LOG("%s Param NULL Err\n",__func__);
        return -EPERM;
    }

    *info = scene_info[ camsensor_drv_ctrl->scene_mode ];

    return TRUE;
}

static int L_camsensor_drv_get_exif_info
(
  common_exif_info_t *exif
)
{
    int rc = TRUE;

    if(exif == NULL)
    {
        ERR_LOG("%s Param NULL Err\n",__func__);
        return -EPERM;
    }

    exif->iso_speed     = camsensor_drv_ctrl->iso_speed;

    exif->exposure_time = camsensor_drv_ctrl->exposure_time;

    exif->focal_length  = EXIF_FOCAL_LENGTH;

    exif->focal_length_35mm = EXIF_FOCAL_LENGTH_35MM;

    exif->f_number      = EXIF_F_NUMBER;

    exif->flash         = camsensor_drv_ctrl->flash_exif_data;

    return rc;
}

static int L_camsensor_drv_get_sensor_exif_info( void )
{
    int rc = 0;
    int i  = 0;
    int iso_data_num = 0;
    unsigned short tmp;
    unsigned short gain_val;
    unsigned long  exptime;

    rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x03, 0x20, BYTE_LEN );
    rc |= L_camsensor_drv_i2c_read( camsensor_drv_client->addr, 0x20, &tmp, BYTE_LEN );
    exptime = ( tmp & 0xFF ) << 24;
    rc |= L_camsensor_drv_i2c_read( camsensor_drv_client->addr, 0x21, &tmp, BYTE_LEN );
    exptime = exptime | (( tmp & 0xFF ) << 16 );
    rc |= L_camsensor_drv_i2c_read( camsensor_drv_client->addr, 0x22, &tmp, BYTE_LEN );
    exptime = exptime | (( tmp & 0xFF ) << 8 );
    rc |= L_camsensor_drv_i2c_read( camsensor_drv_client->addr, 0x23, &tmp, BYTE_LEN );
    exptime = exptime | ( tmp & 0xFF );

    if( camsensor_drv_ctrl->flash_on_flg )
    {
        switch(camsensor_drv_ctrl->scene_mode)
        {
            case CAM_SCENE_NIGHT_PORTRAIT:
              if(exptime > MAX_EXPTIME_NIGTH_VALUE)
              {
                  exptime = MAX_EXPTIME_NIGTH_VALUE;
              }
              break;
            default:
              if(exptime > MAX_EXPTIME_NORMAL_VALUE)
              {
                  exptime = MAX_EXPTIME_NORMAL_VALUE;
              }
              break;
        }
    }

    exptime = exptime / 36;

    camsensor_drv_ctrl->exposure_time = exptime;

    if( rc < 0 )
    {
        ERR_LOG("Get Exposure Time I2C Err\n");
        return rc;
    }

    rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x03, 0x20, BYTE_LEN );
    rc |= L_camsensor_drv_i2c_read( camsensor_drv_client->addr, 0x50, &tmp, BYTE_LEN );

    if( rc < 0 )
    {
        ERR_LOG("Get ISO Speed I2C Err\n");
        return rc;
    }

    if( camsensor_drv_ctrl->flash_on_flg )
    {
        tmp = camsensor_drv_ctrl->again;

    }

    gain_val     = (tmp * 1000 / 32) + 250;
    iso_data_num = ARRAY_SIZE(camsensor_iso_conversion_tbl);

    for(i = 0; i < iso_data_num; i++)
    {
        if( (camsensor_iso_conversion_tbl[i].min <= gain_val)
         && (camsensor_iso_conversion_tbl[i].max > gain_val))
        {
            camsensor_drv_ctrl->iso_speed = camsensor_iso_conversion_tbl[i].value;
            break;
        }
        else if( MAX_GAIN_VALUE <= gain_val)
        {
            camsensor_drv_ctrl->iso_speed = 800;
            break;
        }
    }

    switch(camsensor_drv_ctrl->flash_mode)
    {
        case CAM_FLASH_OFF:
            camsensor_drv_ctrl->flash_exif_data = EXIF_FLASH_OFF;
            break;
        case CAM_FLASH_ON:
            camsensor_drv_ctrl->flash_exif_data = EXIF_FLASH_ON;
            break;
        case CAM_FLASH_AUTO:
            if( camsensor_drv_ctrl->flash_on_flg )
            {
                camsensor_drv_ctrl->flash_exif_data = EXIF_FLASH_AUTO_ON;
            }
            else
            {
                camsensor_drv_ctrl->flash_exif_data = EXIF_FLASH_AUTO_OFF;
            }
            break;
        default:
            ERR_LOG("Flash Mode invalid param error\n");
            camsensor_drv_ctrl->flash_exif_data = EXIF_FLASH_OFF;
            break;
    }

    return rc;
}

#ifdef FEATURE_KYOCERA_DLCHK_GRADATION_PATTERN

static int32_t L_camsensor_drv_dataline_check_config
(
  int mode
)
{
    int32_t rc = 0;
#if(0)
    struct camsensor_drv_i2c_reg_conf const* set_reg;
    uint16_t set_reg_size;

    L_camsensor_drv_pwd_config(0);

    mdelay(300);

    L_camsensor_drv_pwd_config(1);

    set_reg = &camsensor_drv_regs.dataline_check_mode[0];
    set_reg_size = camsensor_drv_regs.dataline_check_mode_reg_settings_size;

    rc = L_camsensor_drv_i2c_write_table(set_reg, set_reg_size);
    mdelay(300);

    camsensor_drv_ctrl->sensormode = mode;

#endif
    return rc;
}
#endif

#ifdef FEATURE_KYOCERA_DLCHK_UVFIX_PATTERN

static int32_t L_camsensor_drv_set_dataline_check_mode( common_dl_check_t* dl_check)
{
    int32_t rc = 0;

    if( dl_check == NULL )
    {
        ERR_LOG("Parm NULL Err\n");
        return -1;
    }

    rc = L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x03, 0x10, BYTE_LEN );
    if( rc < 0 )
    {
        ERR_LOG("I2C Err\n");
        return -1;
    }

    rc = L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x12, 0x13, BYTE_LEN );
    if( rc < 0 )
    {
        ERR_LOG("ISPCTL3 Err\n");
        return -1;
    }

    ERR_LOG("UCON:0x%02x\n", dl_check->parm1);
    ERR_LOG("VCON:0x%02x\n", dl_check->parm2);

    rc = L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x44, dl_check->parm1, BYTE_LEN );
    if( rc < 0 )
    {
        ERR_LOG("UCON Err\n");
        return -1;
    }

    rc = L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x45, dl_check->parm2, BYTE_LEN );
    if( rc < 0 )
    {
        ERR_LOG("VCON Err\n");
        return -1;
    }

    return rc;
}
#endif

static int L_camsensor_drv_pmic_flash_ctrl( camsensor_drv_flash_type flash_mode )
{
    int32_t rc = 0;
    uint32_t mA = 0;

    if( flash_mode == CAMERA_DRV_FLASH_OFF )
    {

        rc = pm8058_set_flash_led_current( PMIC8058_ID_FLASH_LED_0, 0 );

        rc |= pmic_flash_led_set_current( 0 );

        kcj_dev_info_update_flash_led(false);
        if( rc < 0 )
        {
            ERR_LOG("PMIC Set 0mA Err\n");
            return -1;
        }

        msm_5vcnt_exclusion( false, MSM_5VCNT_USER_CAMERA );
    }
    else
    {
        if( flash_mode == CAMERA_DRV_VIDEO_LIGHT )
        {
            mA = 40;
        }
        else if( flash_mode == CAMERA_DRV_MAIN_FLASH )
        {

            if( camsensor_drv_ctrl->flash_up_flg == 1 )
            {
                mA = 100;
            }
            else
            {
                mA = 40;
            }

        }
        else
        {
            mA = 0;
        }

        msm_5vcnt_exclusion( true, MSM_5VCNT_USER_CAMERA );

        if( flash_mode == CAMERA_DRV_MAIN_FLASH )
        {
            rc = pmic_flash_led_set_current( 1 );
        }

        rc = pm8058_set_flash_led_current( PMIC8058_ID_FLASH_LED_0, mA );

        if( flash_mode != CAMERA_DRV_MAIN_FLASH )
        {
            rc |= pmic_flash_led_set_current( 0 );
        }

        if( flash_mode == CAMERA_DRV_VIDEO_LIGHT )
        {
            kcj_dev_info_update_flash_led(true);
        }

        if( rc < 0 )
        {
            ERR_LOG("PMIC Set%dmA Err\n", mA );

            msm_5vcnt_exclusion( false, MSM_5VCNT_USER_CAMERA );

            return -1;
        }
    }

    return rc;
}

static int32_t L_camsensor_drv_set_led_setting( camsensor_drv_led_setting_fase fase )
{
    int32_t rc = 0;
    unsigned short tmp;
    static struct camsensor_drv_led_setting_st led_ctrl;

    switch( fase )
    {
        case LED_SETTING_JUDGE_AUTOFLASH:

            rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x03, 0x20, BYTE_LEN );
            rc |= L_camsensor_drv_i2c_read( camsensor_drv_client->addr, 0xA4, &tmp, BYTE_LEN );
            led_ctrl.exptime = ( tmp & 0xFF ) << 24;
            rc |= L_camsensor_drv_i2c_read( camsensor_drv_client->addr, 0xA5, &tmp, BYTE_LEN );
            led_ctrl.exptime = led_ctrl.exptime | (( tmp & 0xFF ) << 16 );
            rc |= L_camsensor_drv_i2c_read( camsensor_drv_client->addr, 0xA6, &tmp, BYTE_LEN );
            led_ctrl.exptime = led_ctrl.exptime | (( tmp & 0xFF ) << 8 );
            rc |= L_camsensor_drv_i2c_read( camsensor_drv_client->addr, 0xA7, &tmp, BYTE_LEN );
            led_ctrl.exptime = led_ctrl.exptime | ( tmp & 0xFF );
            rc |= L_camsensor_drv_i2c_read( camsensor_drv_client->addr, 0xAB, &led_ctrl.again, BYTE_LEN );

            if( camsensor_drv_ctrl->flash_mode == CAM_FLASH_AUTO )
            {
                if( led_ctrl.again >= 0xA0 && led_ctrl.exptime >= 0x325AA0 )
                {
                    camsensor_drv_ctrl->flash_on_flg = 1;
                }
                else
                {
                    camsensor_drv_ctrl->flash_on_flg = 0;
                }
            }
            else
            {
                camsensor_drv_ctrl->flash_on_flg = 1;
            }
            break;

        case LED_SETTING_BEFORE_PREFLASH:

            rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x03, 0xC4, BYTE_LEN );
            rc |= L_camsensor_drv_i2c_read( camsensor_drv_client->addr, 0x10, &led_ctrl.aectl, BYTE_LEN );
            rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x03, 0xC5, BYTE_LEN );
            rc |= L_camsensor_drv_i2c_read( camsensor_drv_client->addr, 0x10, &led_ctrl.awbctl, BYTE_LEN );
            rc |= L_camsensor_drv_i2c_read( camsensor_drv_client->addr, 0x12, &led_ctrl.awbctl3, BYTE_LEN );

            rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x03, 0xC5, BYTE_LEN );
            rc |= L_camsensor_drv_i2c_read( camsensor_drv_client->addr, 0x15, &led_ctrl.traceCntTh, BYTE_LEN );
            rc |= L_camsensor_drv_i2c_read( camsensor_drv_client->addr, 0x16, &led_ctrl.traceStepLmt, BYTE_LEN );

            rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x03, 0x16, BYTE_LEN );
            rc |= L_camsensor_drv_i2c_read( camsensor_drv_client->addr, 0xB0, &led_ctrl.awb_rgain, BYTE_LEN );
            rc |= L_camsensor_drv_i2c_read( camsensor_drv_client->addr, 0xB1, &led_ctrl.awb_bgain, BYTE_LEN );

            rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x03, 0xC4, BYTE_LEN );
            rc |= L_camsensor_drv_i2c_read( camsensor_drv_client->addr, 0x14, &led_ctrl.ylvl_c414, BYTE_LEN );

            rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x03, 0x20, BYTE_LEN );
            rc |= L_camsensor_drv_i2c_read( camsensor_drv_client->addr, 0x80, &led_ctrl.ylvl_bofore, BYTE_LEN );
            rc |= L_camsensor_drv_i2c_read( camsensor_drv_client->addr, 0xB1, &led_ctrl.ymean_bofore, BYTE_LEN );

            rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x03, 0xC4, BYTE_LEN );
            rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x10, ( led_ctrl.aectl & 0x7F ), BYTE_LEN );
            rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x03, 0xC5, BYTE_LEN );
            rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x10, ( led_ctrl.awbctl | 0x04 ), BYTE_LEN );
            rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x12, ( led_ctrl.awbctl3 & 0x7F ), BYTE_LEN );
            rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x15, 0x02, BYTE_LEN );
            rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x16, 0x40, BYTE_LEN );
            break;

        case LED_SETTING_CALCULATE_VALUE:

            rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x03, 0xC4, BYTE_LEN );
            rc |= L_camsensor_drv_i2c_read( camsensor_drv_client->addr, 0x14, &led_ctrl.ylvl_c414, BYTE_LEN );

            rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x03, 0x20, BYTE_LEN );
            rc |= L_camsensor_drv_i2c_read( camsensor_drv_client->addr, 0x80, &led_ctrl.ylvl, BYTE_LEN );
            rc |= L_camsensor_drv_i2c_read( camsensor_drv_client->addr, 0xB1, &led_ctrl.ymean, BYTE_LEN );

            rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x03, 0xC4, BYTE_LEN );
            rc |= L_camsensor_drv_i2c_read( camsensor_drv_client->addr, 0x66, &led_ctrl.hz, BYTE_LEN );
            if(( led_ctrl.hz & 0x80 ) == 0x80 )
            {
                rc |= L_camsensor_drv_i2c_read( camsensor_drv_client->addr, 0x46, &tmp, BYTE_LEN );
                led_ctrl.anti_band = ( tmp & 0xFF ) << 24;
                rc |= L_camsensor_drv_i2c_read( camsensor_drv_client->addr, 0x47, &tmp, BYTE_LEN );
                led_ctrl.anti_band = led_ctrl.anti_band | (( tmp & 0xFF ) << 16 );
                rc |= L_camsensor_drv_i2c_read( camsensor_drv_client->addr, 0x48, &tmp, BYTE_LEN );
                led_ctrl.anti_band = led_ctrl.anti_band | (( tmp & 0xFF ) << 8 );
                rc |= L_camsensor_drv_i2c_read( camsensor_drv_client->addr, 0x49, &tmp, BYTE_LEN );
                led_ctrl.anti_band = led_ctrl.anti_band | ( tmp & 0xFF );
            }
            else
            {
                rc |= L_camsensor_drv_i2c_read( camsensor_drv_client->addr, 0x4A, &tmp, BYTE_LEN );
                led_ctrl.anti_band = ( tmp & 0xFF ) << 24;
                rc |= L_camsensor_drv_i2c_read( camsensor_drv_client->addr, 0x4B, &tmp, BYTE_LEN );
                led_ctrl.anti_band = led_ctrl.anti_band | (( tmp & 0xFF ) << 16 );
                rc |= L_camsensor_drv_i2c_read( camsensor_drv_client->addr, 0x4C, &tmp, BYTE_LEN );
                led_ctrl.anti_band = led_ctrl.anti_band | (( tmp & 0xFF ) << 8 );
                rc |= L_camsensor_drv_i2c_read( camsensor_drv_client->addr, 0x4D, &tmp, BYTE_LEN );
                led_ctrl.anti_band = led_ctrl.anti_band | ( tmp & 0xFF );
            }

            rc |= L_camsensor_drv_set_led_setting_calc( led_ctrl.exptime, led_ctrl.again, led_ctrl.ylvl, led_ctrl.ymean, led_ctrl.anti_band, &led_ctrl.new_exptime, &led_ctrl.new_again );
            camsensor_drv_ctrl->again = led_ctrl.new_again;

            if(( led_ctrl.ymean * 10 / led_ctrl.ymean_bofore > 30 ) && (( led_ctrl.ymean * 2 - led_ctrl.ymean_bofore ) * 10 > led_ctrl.ylvl_bofore * 9 ))
            {
                camsensor_drv_ctrl->flash_up_flg = 0;

            }
            else
            {
                camsensor_drv_ctrl->flash_up_flg = 1;

            }

            rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x03, 0x20, BYTE_LEN );
            rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x20, (( led_ctrl.new_exptime >> 24 ) & 0xFF ), BYTE_LEN );
            rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x21, (( led_ctrl.new_exptime >> 16 ) & 0xFF ), BYTE_LEN );
            rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x22, (( led_ctrl.new_exptime >> 8 ) & 0xFF ), BYTE_LEN );
            rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x23, ( led_ctrl.new_exptime & 0xFF ), BYTE_LEN );
            rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x50, led_ctrl.new_again, BYTE_LEN );
            rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x03, 0xC5, BYTE_LEN );
            rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x10, ( led_ctrl.awbctl & 0x7F ), BYTE_LEN );
            break;

        case LED_SETTING_RETURN_REGISTER:

            rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x03, 0xC5, BYTE_LEN );
            rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x12, led_ctrl.awbctl3, BYTE_LEN );
            rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x15, led_ctrl.traceCntTh, BYTE_LEN );
            rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x16, led_ctrl.traceStepLmt, BYTE_LEN );
            rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x03, 0x20, BYTE_LEN );
            rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x20, (( led_ctrl.exptime >> 24 ) & 0xFF ), BYTE_LEN );
            rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x21, (( led_ctrl.exptime >> 16 ) & 0xFF ), BYTE_LEN );
            rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x22, (( led_ctrl.exptime >> 8 ) & 0xFF ), BYTE_LEN );
            rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x23, ( led_ctrl.exptime & 0xFF ), BYTE_LEN );
            rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x50, led_ctrl.again, BYTE_LEN );
            rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x03, 0xC4, BYTE_LEN );
            rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x10, led_ctrl.aectl, BYTE_LEN );
            rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x03, 0xC5, BYTE_LEN );
            rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x10, led_ctrl.awbctl, BYTE_LEN );
            rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0x03, 0x16, BYTE_LEN );
            rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0xB0, led_ctrl.awb_rgain, BYTE_LEN );
            rc |= L_camsensor_drv_i2c_write( camsensor_drv_client->addr, 0xB1, led_ctrl.awb_bgain, BYTE_LEN );

            break;

        default:

            rc = -EINVAL;
            break;
    }

    return rc;
}

static int32_t L_camsensor_drv_set_led_setting_calc( unsigned long exptime, unsigned short again, unsigned short ylvl, unsigned short ymean, unsigned long anti, unsigned long*  new_exptime, unsigned short* new_again )
{
    unsigned long  div_ratio;

#if defined(CONFIG_TARGET_PRODUCT_C5155)
    unsigned long  div_ratio_normal;
#endif

    unsigned long  div_ag_ratio;
    unsigned short band_pos;
    unsigned char  int_update = 1;

    if( new_exptime == NULL || new_again == NULL )
    {
        ERR_LOG("%s Param NULL Err\n",__func__);
        return -EPERM;
    }

    if( ymean == 0 )
    {
        ymean = 1;
    }

#if defined(CONFIG_TARGET_PRODUCT_C5155)
    div_ratio_normal = ( ylvl * 10000 ) / ymean;
    div_ratio = (div_ratio_normal/100)*(div_ratio_normal/100);
#else
    div_ratio = ( ylvl * 10000 ) / ymean;
#endif

    div_ag_ratio = ( 0x28 * 10000 ) / again;
    if(div_ratio > 300000L)
    {
        ERR_LOG("again calc overflow?\n");
        ERR_LOG("again : %d\n",again);
        ERR_LOG("ylvl  : %d\n",ylvl);
        ERR_LOG("ymean : %d\n",ymean);
    }
    if( ylvl < ymean )
    {

#if defined(CONFIG_TARGET_PRODUCT_C5155)
        if( div_ratio_normal > div_ag_ratio )
#else
        if( div_ratio > div_ag_ratio )
#endif

        {
            int_update = 0;
            *new_again = ( unsigned short )((( unsigned long )again * div_ratio ) / 10000 );
        }
        else
        {
            int_update = 1;
            *new_again = ( unsigned short )((( unsigned long )again * div_ag_ratio ) / 10000 );

#if defined(CONFIG_TARGET_PRODUCT_C5155)
            div_ratio = (( div_ratio * 100 ) / div_ag_ratio) * 100;
#else
            div_ratio = ( div_ratio * 10000 ) / div_ag_ratio;
#endif
        }
    }
    else
    {
        *new_again = again;
    }

    if( int_update )
    {

        unsigned long div_ratio_u;
        unsigned long div_ratio_m;
        unsigned long div_ratio_d;
        div_ratio_u = div_ratio / 100;
        div_ratio_m = div_ratio_u % 100;
        div_ratio_u = div_ratio_u / 100;
        div_ratio_d = div_ratio % 100;
        *new_exptime = exptime * div_ratio_d / 100;
        *new_exptime += exptime * div_ratio_m;
        *new_exptime /= 100;
        *new_exptime += exptime * div_ratio_u;

        band_pos = *new_exptime / anti;
        if( band_pos > 0 )
        {
            div_ratio = ( *new_exptime / band_pos * 100 ) / ( anti / 100 );
            *new_again = ( *new_again * div_ratio ) / 10000;
            *new_exptime = band_pos * anti;
        }
    }
    else
    {
        *new_exptime = exptime;
    }

    if( *new_again < 0x28 )
    {
        *new_again = 0x28;
    }
    if( *new_again > 0xC0 )
    {
        *new_again = 0xC0;
    }

    return 0;
}

static int32_t L_camsensor_drv_set_sensor_mode
(
  int mode,
  int res
)
{
    int32_t rc = 0;

    struct camsensor_drv_i2c_reg_conf const* set_reg;
    uint16_t set_reg_size;

    switch (mode)
    {
        case SENSOR_PREVIEW_MODE:
            if( camsensor_drv_ctrl->sensormode == mode )
            {
                ERR_LOG("Allready Preview Mode\n");
                break;
            }

            set_reg = &camsensor_drv_regs.preview_reg_settings[0];
            set_reg_size = camsensor_drv_regs.preview_reg_settings_size;

            rc = L_camsensor_drv_i2c_write_table( set_reg, set_reg_size );
            if( rc < 0 )
            {
                ERR_LOG("Preview Reg Setting I2C Err\n");
                break;
            }

            camsensor_drv_ctrl->sensormode = SENSOR_PREVIEW_MODE;

            break;

        case SENSOR_SNAPSHOT_MODE:

            rc = L_camsensor_drv_wait_vsync(1);
            if( rc < 0 )
            {
                ERR_LOG("SENSOR_SNAPSHOT_MODE 1 Wait Vsync High Err\n");
                return rc;
            }

            if( camsensor_drv_ctrl->flash_on_flg == 1 )
            {

                rc = L_camsensor_drv_set_led_setting( LED_SETTING_CALCULATE_VALUE );
                if( rc < 0 )
                {
                    ERR_LOG("SENSOR_SNAPSHOT_MODE LED Setting Err\n");
                    return rc;
                }

                rc = L_camsensor_drv_pmic_flash_ctrl( CAMERA_DRV_MAIN_FLASH );
                if( rc < 0 )
                {
                    ERR_LOG("SENSOR_SNAPSHOT_MODE PMIC Main Flash Ctrl Err\n");
                    break;
                }
            }

            set_reg = &camsensor_drv_regs.still_reg_settings[0];
            set_reg_size = camsensor_drv_regs.still_reg_settings_size;

            rc = L_camsensor_drv_i2c_write_table( set_reg, set_reg_size );
            if( rc < 0 )
            {
                ERR_LOG("Snapshot Reg Setting I2C Err\n");
                break;
            }

            rc = L_camsensor_drv_change_irq( VSYNC_IRQ_LOW );
            if( rc < 0 )
            {
                ERR_LOG("L_camsensor_drv_prepare_snapshot L_camsensor_drv_change_irq()Err\n");
                return rc;
            }

            L_camsensor_drv_vsync_proc_entry( KTHREAD_PROC_STILL_FRAME );

            rc = L_camsensor_drv_kthread_run( 1 );
            if( rc < 0 )
            {
                ERR_LOG("SENSOR_SNAPSHOT_MODE L_camsensor_drv_kthread_run() Err\n");
                break;
            }

            camsensor_drv_ctrl->sensormode = SENSOR_SNAPSHOT_MODE;

            break;

        case SENSOR_RAW_SNAPSHOT_MODE:
            break;
#ifdef FEATURE_KYOCERA_DLCHK_GRADATION_PATTERN
        case SENSOR_DATALINE_CHECK_MODE:
            rc = L_camsensor_drv_dataline_check_config(mode);
            break;
#endif
        case SENSOR_STILL_TO_PREVIEW_MODE:

            set_reg = &camsensor_drv_regs.preview_reg_settings[0];
            set_reg_size = camsensor_drv_regs.preview_reg_settings_size;

            rc = L_camsensor_drv_i2c_write_table( set_reg, set_reg_size );
            if( rc < 0 )
            {
                ERR_LOG("SENSOR_STILL_TO_PREVIEW_MODE Preview Reg Setting I2C Err\n");
                break;
            }

            if(camsensor_drv_ctrl->scene_mode != CAM_SCENE_AUTO)
            {
                rc = L_camsensor_drv_set_scene(camsensor_drv_ctrl->scene_mode,true);
                if( rc < 0 )
                {
                    ERR_LOG("SENSOR_STILL_TO_PREVIEW_MODE Preview Reg Setting I2C Err\n");
                    break;
                }
            }

            if( camsensor_drv_ctrl->flash_on_flg == 1 )
            {
                camsensor_drv_ctrl->flash_on_flg = 0;
                rc = L_camsensor_drv_set_led_setting( LED_SETTING_RETURN_REGISTER );
                if( rc < 0 )
                {
                    ERR_LOG("SENSOR_PREVIEW_MODE LED Setting Err\n");
                    return rc;
                }
            }

            camsensor_drv_ctrl->sensormode = SENSOR_PREVIEW_MODE;

            break;
        default:
            ERR_LOG("%s Param Err [%d]\n",__func__ ,mode);
            rc = -EINVAL;
            break;
    }

    if (rc < 0)
    {
        ERR_LOG("%s : ERR [%d]\n",__func__ , rc);
    }

    return rc;
}

static int32_t L_camsensor_drv_prepare_snapshot
(
  void
)
{
    int32_t rc = 0;

    rc = L_camsensor_drv_change_irq( VSYNC_IRQ_HIGH );
    if( rc < 0 )
    {
        ERR_LOG("L_camsensor_drv_prepare_snapshot L_camsensor_drv_change_irq()Err\n");
        return rc;
    }

    if( camsensor_drv_ctrl->flash_mode != CAM_FLASH_ON && camsensor_drv_ctrl->flash_mode != CAM_FLASH_AUTO )
    {
        camsensor_drv_ctrl->flash_on_flg = 0;
        return rc;
    }

    rc = L_camsensor_drv_wait_vsync(1);
    if( rc < 0 )
    {
        ERR_LOG("L_camsensor_drv_prepare_snapshot 1 Wait Vsync High Err\n");
        return rc;
    }

    rc = L_camsensor_drv_set_led_setting( LED_SETTING_JUDGE_AUTOFLASH );
    if( rc < 0 )
    {
        ERR_LOG("L_camsensor_drv_prepare_snapshot LED Setting Err\n");
        return rc;
    }

    if( camsensor_drv_ctrl->flash_on_flg == 0 )
    {
        return rc;
    }

    rc = L_camsensor_drv_set_led_setting( LED_SETTING_BEFORE_PREFLASH );
    if( rc < 0 )
    {
        ERR_LOG("L_camsensor_drv_prepare_snapshot LED Setting Err\n");
        return rc;
    }

    rc = L_camsensor_drv_pmic_flash_ctrl( CAMERA_DRV_VIDEO_LIGHT );
    if( rc < 0 )
    {
        ERR_LOG("L_camsensor_drv_prepare_snapshot PMIC Pre Flash Ctrl Err\n");
        return rc;
    }

    rc = L_camsensor_drv_wait_vsync(4);

    if( rc < 0 )
    {
        ERR_LOG("L_camsensor_drv_prepare_snapshot 5 Wait Vsync High Err\n");
        return rc;
    }

    return rc;
}

static int L_camsensor_drv_set_wb
(
  camera_wb_type wb
)
{
    int rc = TRUE;
    struct camsensor_drv_i2c_reg_conf const* set_reg;
    uint16_t set_reg_size;

    if( wb >= CAM_WB_MAX )
    {
        ERR_LOG("[%s] Param Err [%d]\n",__func__ ,wb);
        return -EINVAL;
    }

    if( camsensor_drv_ctrl->wb_mode == wb )
    {

        return TRUE;
    }

    switch( wb )
    {
        case CAM_WB_AUTO:
            set_reg = &camsensor_drv_regs.set_wb_auto_reg[0];
            set_reg_size = camsensor_drv_regs.set_wb_reg_size;
            break;
        case CAM_WB_DAYLIGHT:
            set_reg = &camsensor_drv_regs.set_wb_daylight_reg[0];
            set_reg_size = camsensor_drv_regs.set_wb_reg_size;
            break;
        case CAM_WB_CLOUDY_DAYLIGHT:
            set_reg = &camsensor_drv_regs.set_wb_cloudy_reg[0];
            set_reg_size = camsensor_drv_regs.set_wb_reg_size;
            break;
        case CAM_WB_INCANDESCENT:
            set_reg = &camsensor_drv_regs.set_wb_incandescent_reg[0];
            set_reg_size = camsensor_drv_regs.set_wb_reg_size;
            break;
        case CAM_WB_FLUORESCENT:
            set_reg = &camsensor_drv_regs.set_wb_fluorescent_reg[0];
            set_reg_size = camsensor_drv_regs.set_wb_reg_size;
            break;
        default:
            ERR_LOG("%s Param Err", __func__);
            return -EINVAL;
    }

    rc = L_camsensor_drv_set_parm_reg_table(set_reg, set_reg_size);
    if( rc < 0 )
    {
        ERR_LOG("failed camsensor_drv_i2c_write! (rc=%d)\n",rc);
        return rc;
    }

    camsensor_drv_ctrl->wb_mode = wb;

    return rc;
}

static int L_camsensor_drv_set_effect
(
  camera_effect_type effect
)
{
    int rc = TRUE;
    struct camsensor_drv_i2c_reg_conf const* set_reg;
    uint16_t set_reg_size;

    if( effect >= CAM_EFFECT_MAX )
    {
        ERR_LOG("%s Param Err [%d]\n",__func__ ,effect);
        return -EFAULT;
    }

    if( camsensor_drv_ctrl->effect_mode == effect )
    {

        return TRUE;
    }

    switch( effect )
    {
        case CAM_EFFECT_NONE:
            set_reg = &camsensor_drv_regs.set_effect_none_reg[0];
            set_reg_size = camsensor_drv_regs.set_effect_reg_size;
            break;
        case CAM_EFFECT_MONO:
            set_reg = &camsensor_drv_regs.set_effect_mono_reg[0];
            set_reg_size = camsensor_drv_regs.set_effect_reg_size;
            break;
        case CAM_EFFECT_SEPIA:
            set_reg = &camsensor_drv_regs.set_effect_sepia_reg[0];
            set_reg_size = camsensor_drv_regs.set_effect_reg_size;
            break;
        case CAM_EFFECT_NEGA:
            set_reg = &camsensor_drv_regs.set_effect_negative_reg[0];
            set_reg_size = camsensor_drv_regs.set_effect_reg_size;
            break;
        case CAM_EFFECT_AQUA:
            set_reg = &camsensor_drv_regs.set_effect_aqua_reg[0];
            set_reg_size = camsensor_drv_regs.set_effect_reg_size;
            break;
        default:
            ERR_LOG("%s Param Err [%d]\n",__func__ ,effect);
            return -EFAULT;
    }

    rc = L_camsensor_drv_set_parm_reg_table(set_reg, set_reg_size);
    if( rc < 0 )
    {
        ERR_LOG("failed camsensor_drv_i2c_write! (rc=%d)\n",rc);
        return rc;
    }

    camsensor_drv_ctrl->effect_mode = effect;

    return rc;
}

static int L_camsensor_drv_set_ae_mode
(
  camera_auto_exposure_type aec_mode
)
{
    int rc = TRUE;
    struct camsensor_drv_i2c_reg_conf const* set_reg;
    uint16_t set_reg_size;

    if( aec_mode >= CAM_AEC_MAX )
    {
        ERR_LOG("%s Param Err [%d]",__func__ ,aec_mode);
        rc = -EINVAL;
        return rc;
    }

    if( camsensor_drv_ctrl->aec_mode == aec_mode )
    {

        return TRUE;
    }

    switch( aec_mode )
    {
        case CAM_AEC_AVERAGE:
            set_reg = &camsensor_drv_regs.set_ae_mode_average_reg[0];
            set_reg_size = camsensor_drv_regs.set_ae_mode_reg_size;
            break;
        case CAM_AEC_CENTER:
            set_reg = &camsensor_drv_regs.set_ae_mode_center_reg[0];
            set_reg_size = camsensor_drv_regs.set_ae_mode_reg_size;
            break;
        case CAM_AEC_SPOT:
            set_reg = &camsensor_drv_regs.set_ae_mode_spot_reg[0];
            set_reg_size = camsensor_drv_regs.set_ae_mode_reg_size;
            break;
        default:
            ERR_LOG("%s Param Err [%d]",__func__ ,aec_mode);
            return -EINVAL;
    }

    rc = L_camsensor_drv_set_parm_reg_table(set_reg, set_reg_size);
    if( rc < 0 )
    {
        ERR_LOG("failed camsensor_drv_i2c_write! (rc=%d)\n",rc);
        return rc;
    }

    camsensor_drv_ctrl->aec_mode = aec_mode;

    return rc;
}

static int L_camsensor_drv_set_scene
(
  camera_scene_type scene_mode,
  int write_mode
)
{
    int rc = TRUE;
    struct camsensor_drv_i2c_reg_conf const* set_reg;
    uint16_t set_reg_size;

    if( scene_mode >= CAM_SCENE_MAX )
    {
        ERR_LOG("%s Param Err [%d] \n",__func__ ,scene_mode);
        return -EINVAL;
    }

    if( camsensor_drv_ctrl->scene_mode == scene_mode && !write_mode )
    {

        return TRUE;
    }

    switch( scene_mode )
    {

        case CAM_SCENE_AUTO:
            set_reg = &camsensor_drv_regs.set_scene_auto_reg[0];
            set_reg_size = camsensor_drv_regs.set_scene_auto_reg_size;;
            break;

        case CAM_SCENE_PORTRAIT:
            set_reg = &camsensor_drv_regs.set_scene_portrait_reg[0];
            set_reg_size = camsensor_drv_regs.set_scene_portrait_reg_size;
            camsensor_drv_ctrl->wb_mode    = CAM_WB_AUTO;
            camsensor_drv_ctrl->aec_mode   = CAM_AEC_CENTER;
            camsensor_drv_ctrl->flash_mode = CAM_FLASH_AUTO;
            break;

        case CAM_SCENE_SCENERY:
            set_reg = &camsensor_drv_regs.set_scene_scenery_reg[0];
            set_reg_size = camsensor_drv_regs.set_scene_scenery_reg_size;
            camsensor_drv_ctrl->wb_mode    = CAM_WB_AUTO;
            camsensor_drv_ctrl->aec_mode   = CAM_AEC_AVERAGE;
            camsensor_drv_ctrl->flash_mode = CAM_FLASH_AUTO;
            break;

        case CAM_SCENE_NIGHT_PORTRAIT:
            set_reg = &camsensor_drv_regs.set_scene_night_portrait_reg[0];
            set_reg_size = camsensor_drv_regs.set_scene_night_portrait_reg_size;
            camsensor_drv_ctrl->wb_mode    = CAM_WB_AUTO;
            camsensor_drv_ctrl->aec_mode   = CAM_AEC_CENTER;
            camsensor_drv_ctrl->flash_mode = CAM_FLASH_ON;
            break;

        case CAM_SCENE_NIGHT_SCENERY:
            set_reg = &camsensor_drv_regs.set_scene_night_scenery_reg[0];
            set_reg_size = camsensor_drv_regs.set_scene_night_scenery_reg_size;
            camsensor_drv_ctrl->wb_mode    = CAM_WB_AUTO;
            camsensor_drv_ctrl->aec_mode   = CAM_AEC_AVERAGE;
            camsensor_drv_ctrl->flash_mode = CAM_FLASH_OFF;
            break;

        case CAM_SCENE_ACTION:
            set_reg = &camsensor_drv_regs.set_scene_action_reg[0];
            set_reg_size = camsensor_drv_regs.set_scene_action_reg_size;
            camsensor_drv_ctrl->wb_mode    = CAM_WB_AUTO;
            camsensor_drv_ctrl->aec_mode   = CAM_AEC_CENTER;
            camsensor_drv_ctrl->flash_mode = CAM_FLASH_OFF;
            break;

        default:
            ERR_LOG("%s Param Err [%d] \n",__func__ ,scene_mode);
            return -EINVAL;
    }

    rc = L_camsensor_drv_set_parm_reg_table(set_reg, set_reg_size);
    if( rc < 0 )
    {
        ERR_LOG("failed camsensor_drv_i2c_write! (rc=%d)\n",rc);
        return rc;
    }

    camsensor_drv_ctrl->scene_mode = scene_mode;

    return rc;
}

static int L_camsensor_drv_sensor_init
(
  const struct msm_camera_sensor_info *data
)
{
    int32_t  rc;

    camsensor_drv_ctrl = kzalloc(sizeof(struct camsensor_drv_ctrl), GFP_KERNEL);
    if (!camsensor_drv_ctrl)
    {
        ERR_LOG("camsensor_drv_init failed!\n");
        rc = -ENOMEM;
        return rc;
    }

    if (data)
    {
        camsensor_drv_ctrl->sensordata = data;
    }
    camsensor_drv_ctrl->camsensor_drv_regs = NULL;
    camsensor_drv_ctrl->prev_res           = S_QTR_SIZE;

    camsensor_drv_ctrl->flash_on_flg    = 0;
    camsensor_drv_ctrl->wb_mode         = CAM_WB_AUTO;
    camsensor_drv_ctrl->effect_mode     = CAM_EFFECT_NONE;
    camsensor_drv_ctrl->scene_mode      = CAM_SCENE_AUTO;
    camsensor_drv_ctrl->exposure_time   = 0;
    camsensor_drv_ctrl->iso_speed       = 0;
    camsensor_drv_ctrl->flash_mode      = CAM_FLASH_OFF;
    camsensor_drv_ctrl->flash_exif_data = EXIF_FLASH_OFF;

    camsensor_drv_ctrl->aec_mode     = CAM_AEC_CENTER;

    camsensor_drv_ctrl->api_after_proc_kind = 0x00000000;

    L_camsensor_drv_vsync_proc_entry(0);

    atomic_set( &camsensor_drv_vsync_irq.v_frame_count, 0 );

    atomic_set( &camsensor_drv_vsync_irq.kthread_frame_count, 0 );

    atomic_set( &start_vsync_count, false );

    atomic_set( &start_kthread_vsync_count, false );

    camsensor_drv_vsync_irq.kthread_frame_cnt = 0;

    rc = L_camsensor_drv_pwd_config(1);
    if (rc < 0)
    {
        ERR_LOG("L_camsensor_drv_pwd_config(1) failed!\n");
        L_camsensor_drv_power_down();
        return rc;
    }

    msm_camio_camif_pad_reg_reset();

    rc = L_camsensor_drv_read_hw_ver();
    if (rc < 0)
    {
        ERR_LOG("L_camsensor_drv_read_hw_ver failed!\n");
        L_camsensor_drv_power_down();
        return rc;
    }

    rc = L_camsensor_drv_set_reg_table();
    if (rc < 0)
    {
        ERR_LOG("L_camsensor_drv_set_reg_table failed!\n");
        L_camsensor_drv_power_down();
        return rc;
    }

    if (camsensor_drv_ctrl->prev_res == S_QTR_SIZE)
    {
        rc = L_camsensor_drv_setting(S_REG_INIT, S_RES_PREVIEW);
        camsensor_drv_ctrl->sensormode = SENSOR_PREVIEW_MODE;
    }
    else
    {
        rc = L_camsensor_drv_setting(S_REG_INIT, S_RES_CAPTURE);
        camsensor_drv_ctrl->sensormode = SENSOR_SNAPSHOT_MODE;
    }

    if (rc < 0)
    {
        ERR_LOG("L_camsensor_drv_setting failed. rc = %d\n", rc);
        L_camsensor_drv_power_down();
    }

    return rc;
}

static int L_camsensor_drv_sensor_release
(
  void
)
{
    int rc = -EBADF;

    mutex_lock(&camsensor_drv_mutex);
    camsensor_kthread_stop_flg = true;
    mutex_unlock(&camsensor_drv_mutex);

    rc = L_camsensor_drv_pmic_flash_ctrl( CAMERA_DRV_FLASH_OFF );
    if( rc < 0 )
    {
        ERR_LOG("L_camsensor_drv_sensor_release PMIC Flash Ctrl Err\n");
    }

    free_irq(gpio_to_irq(CAMSENSOR_VSYNC_GPIO),0);

    L_camsensor_drv_power_down();

    return rc;
}

static int L_camsensor_drv_sensor_config
(
  void __user *argp
)
{
    struct sensor_cfg_data cdata;
    int32_t rc = 0;

    if (copy_from_user(&cdata ,(void *)argp ,sizeof(struct sensor_cfg_data)))
    {
        ERR_LOG("%s Err\n",__func__);
        return -EFAULT;
    }

    mutex_lock(&camsensor_drv_mutex);

    switch( cdata.cfgtype )
    {
        case CFG_SET_MODE:
            rc = L_camsensor_drv_set_sensor_mode( cdata.mode, cdata.rs );
            if( rc < 0 )
            {
                ERR_LOG("Mode Setting Err\n");
            }
            break;
        case CFG_SET_DATALINE_MODE:

#ifdef FEATURE_KYOCERA_DLCHK_GRADATION_PATTERN
            rc = L_camsensor_drv_set_sensor_mode( cdata.mode, cdata.rs )
#endif
#ifdef FEATURE_KYOCERA_DLCHK_UVFIX_PATTERN
            rc = L_camsensor_drv_set_dataline_check_mode( &cdata.dl_check );
#endif

            break;

        case CFG_PREPARE_SNAPSHOT:
            rc = L_camsensor_drv_prepare_snapshot();
            if( rc < 0 )
            {
                ERR_LOG("CFG_PREPARE_SNAPSHOT L_camsensor_drv_prepare_snapshot() Err\n");
            }
            break;

        case CFG_GET_EXIF_INFO:
            rc = L_camsensor_drv_get_exif_info(&(cdata.exif_info));
            if( rc < 0 )
            {
                ERR_LOG("L_camsensor_drv_get_exif_info Err :[%d]\n",rc);
            }
            else
            {

                if( copy_to_user( argp, &cdata, sizeof( struct sensor_cfg_data )))
                {
                    ERR_LOG("CFG_GET_EXIF_INFO copy_to_user() Err\n");
                    rc = -EPERM;
                }
                else
                {
                    rc = TRUE;
                }
            }
            break;

        case CFG_SET_WB:
            rc = L_camsensor_drv_set_wb((camera_wb_type)cdata.cfg.value );
            if( rc < 0 )
            {
                ERR_LOG("L_camsensor_drv_set_wb Err :[%d]\n",rc);
            }
            break;

        case CFG_SET_EFFECT:
            rc = L_camsensor_drv_set_effect((camera_effect_type)cdata.cfg.value );
            if( rc < 0 )
            {
                ERR_LOG("L_camsensor_drv_set_effect Err :[%d]\n",rc);
            }
            break;

        case CFG_SET_EXPOSURE_MODE:
            rc = L_camsensor_drv_set_ae_mode((camera_auto_exposure_type)cdata.cfg.value );
            if( rc < 0 )
            {
                ERR_LOG("camsensor_drv_set_ae_mode Err :[%d]\n",rc);
            }
            break;

        case CFG_GET_SCENE_INFO:
            rc = L_camsensor_drv_get_scene_info(&(cdata.scene_info ));
            if( rc < 0 )
            {
                ERR_LOG("L_camsensor_drv_get_scene_info Err :[%d]\n",rc);
            }
            else
            {

                if( copy_to_user( argp, &cdata, sizeof( struct sensor_cfg_data )))
                {
                    ERR_LOG("CFG_GET_SCENE_INFO copy_to_user() Err\n");
                    rc = -EPERM;
                }
                else
                {
                    rc = TRUE;
                }
            }
            break;

        case CFG_SET_SCENE:
            rc = L_camsensor_drv_set_scene(cdata.mode,false);
            if( rc < 0 )
            {
                ERR_LOG("L_camsensor_drv_set_scene Err :[%d]\n",rc);
            }
            break;

        default:
            break;
    }

    mutex_unlock(&camsensor_drv_mutex);

    return rc;
}

static int L_camsensor_drv_sensor_probe
(
  const struct msm_camera_sensor_info *info,
  struct msm_sensor_ctrl *sensor
)
{
    int rc = 0;

    rc = L_camsensor_drv_probe_init_sensor(info);
    if (rc < 0)
    {
        ERR_LOG("SENSOR PROBE FAILS!\n");
        return rc;
    }

    rc = i2c_add_driver(&camsensor_drv_i2c_driver);
    if (rc < 0)
    {
        ERR_LOG("failed camsensor_drv i2c_add_driver! (rc=%d)\n", rc);
        ERR_LOG("SENSOR PROBE FAILS!\n");
        rc = -ENOTSUPP;
        return rc;
    }

    sensor->s_init = L_camsensor_drv_sensor_init;
    sensor->s_release = L_camsensor_drv_sensor_release;
    sensor->s_config  = L_camsensor_drv_sensor_config;
    sensor->s_camera_type = BACK_CAMERA_2D;
    sensor->s_mount_angle  = 0;

    return rc;
}

static int L_camsensor_drv_probe
(
  struct platform_device *pdev
)
{
    return msm_camera_drv_start(pdev, L_camsensor_drv_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
    .probe = L_camsensor_drv_probe,
    .driver = {
        .name = MCAM_SENSOR_DEVICE_NAME,
        .owner = THIS_MODULE,
    },
};

static int __init camsensor_drv_init
(
  void
)
{
    return platform_driver_register(&msm_camera_driver);
}

module_init(camsensor_drv_init);
