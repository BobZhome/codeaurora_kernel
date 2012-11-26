/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
*/





















/*****************************************************************************/
/* include files                                                             */
/*****************************************************************************/
#include <linux/types.h>
#include <asm/uaccess.h>
#include <linux/kernel.h>
#include <asm/atomic.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/mutex.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <mach/hs_io_ctl_a.h>
#include "apds9900.h"

#include <linux/wakelock.h>


/*****************************************************************************/
/* defines & const                                                           */
/*****************************************************************************/
#define PROXIMITY_DEV_NAME             "proximity"
#define LIGHT_SENSOR_DEFAULT_DELAY        (200)
#define PROXIMITY_SENSOR_DEFAULT_DELAY    (10)
#define RTN_OK                             (0)
#define RTN_NG                             (-1)

#define delay_to_jiffies(d) ((d)?msecs_to_jiffies(d):1)


static bool proximity_log_enable = false;
static bool light_log_enable = false;


#define PROXIMITY_DPRINTK(msg,...) \
    if(proximity_log_enable) printk("[PROXIMITY_SENSOR]:%d " msg,__LINE__,##__VA_ARGS__);

#define LIGHT_DPRINTK(msg,...) \
    if(light_log_enable) printk("[LIGHT_SENSOR]:%d " msg,__LINE__,##__VA_ARGS__);


#undef APDS9900_WAIT_LUX_CHG



#define TYPE_DEFAULT                          (0)
#define TYPE_SFN                              (1)
#define TYPE_LAB                              (2)

struct proximity_threshold{
    int lv;
    int hi;
    int lo;
};






#ifdef CONFIG_TARGET_PRODUCT_C5155
#if defined (CONFIG_TARGET_PROTOTYPE_WS0)

#define REG_PPCOUNT_INIT_VALUE                0x02
#define REG_CONTROL_PDRIVE_INIT_VALUE         0x00
const static struct proximity_threshold proximity_thres_tbl[] = {
    {0,0xffff,360},
    {1,640,0},
};
#elif defined (CONFIG_TARGET_PROTOTYPE_WS1)

#define REG_PPCOUNT_INIT_VALUE                0x01
#define REG_CONTROL_PDRIVE_INIT_VALUE         0x00
const static struct proximity_threshold proximity_thres_tbl[] = {
    {0,0xffff,395},
    {1,575,0},
};
#else

#define REG_PPCOUNT_INIT_VALUE                0x02
#define REG_CONTROL_PDRIVE_INIT_VALUE         0x40
const static struct proximity_threshold proximity_thres_tbl[] = {
    {0,0xffff,950},
    {1,1000,850},
    {2,900,780},
    {3,830,700},
    {4,750,0},
};
#endif
#endif

#ifdef CONFIG_TARGET_PRODUCT_C5170
#if defined (CONFIG_TARGET_PROTOTYPE_WS0)

#define REG_PPCOUNT_INIT_VALUE                0x02
#define REG_CONTROL_PDRIVE_INIT_VALUE         0x00
const static struct proximity_threshold proximity_thres_tbl[] = {
    {0,0xffff,360},
    {1,640,0},
};
#elif defined (CONFIG_TARGET_PROTOTYPE_WS1)

#define REG_PPCOUNT_INIT_VALUE                0x01
#define REG_CONTROL_PDRIVE_INIT_VALUE         0x40
const static struct proximity_threshold proximity_thres_tbl[] = {
    {0,0xffff,775},
    {1,800,0},
};
#else

#define REG_PPCOUNT_INIT_VALUE                0x03
#define REG_CONTROL_PDRIVE_INIT_VALUE         0xC0
const static struct proximity_threshold proximity_thres_tbl[] = {
    {0,0xffff,830},
    {1,850,780},
    {2,800,730},
    {3,750,680},
    {4,700,0},
};
#endif
#endif



#ifdef CONFIG_TARGET_PRODUCT_C5155
#define SFN_REG_PPCOUNT_INIT_VALUE            0x01
#define SFN_REG_CONTROL_PDRIVE_INIT_VALUE     0x00
const static struct proximity_threshold sfn_proximity_thres_tbl[] = {
    {0,0xffff,395},
    {1,575,0},
};
#else
#define SFN_REG_PPCOUNT_INIT_VALUE            0x02
#define SFN_REG_CONTROL_PDRIVE_INIT_VALUE     0x40
const static struct proximity_threshold sfn_proximity_thres_tbl[] = {
    {0,0xffff,875},
    {1,960,0},
};
#endif

#ifdef CONFIG_TARGET_PRODUCT_C5155
#define LAB_REG_PPCOUNT_INIT_VALUE            0x02
#define LAB_REG_CONTROL_PDRIVE_INIT_VALUE     0x00
const static struct proximity_threshold lab_proximity_thres_tbl[] = {
    {0,0xffff,974},
    {1,1022,711},
    {2,759,491},
    {3,539,451},
    {4,469,0},
};
#else
#define LAB_REG_PPCOUNT_INIT_VALUE            0x03
#define LAB_REG_CONTROL_PDRIVE_INIT_VALUE     0x40
const static struct proximity_threshold lab_proximity_thres_tbl[] = {
    {0,0xffff,974},
    {1,1022,756},
    {2,804,661},
    {3,709,636},
    {4,654,0},
};
#endif

/*****************************************************************************/
/* static variable                                                           */
/*****************************************************************************/
/* related to GPIO Interrupt */
const int IRQ_NUM = MSM_GPIO_TO_INT(GPIO_PS_OUT);
int IRQ_LIST_NO = 0;
void *irq_dev_id = (void *)&IRQ_NUM;

/* related to common */
static struct i2c_client *this_client = NULL;
struct delayed_work apds9900_proximity_work;
struct input_dev *input;
struct delayed_work apds9900_light_work;
struct input_dev *input2;

struct proximity_data {
    atomic_t enable;                /* attribute value */
    atomic_t delay;                 /* attribute value */

    int last_data;

};
static struct proximity_data proximity_data_tbl;

struct light_data {
    atomic_t enable;                /* attribute value */
    atomic_t delay;                 /* attribute value */
    int last_data;
};
static struct light_data light_data_tbl;

static struct proximity_threshold *proximity_thres;
static u8 proximity_max_level;


static struct wake_lock proximity_wake_lock;





static u8 reg_value_ptime = 0;
static u8 reg_value_wtime = 0;
static u8 reg_value_atime = 0;
static u8 reg_value_config = 0;
static u8 reg_value_control = 0;
static u8 reg_value_ppcount = 0;
static u16 reg_value_on_thres = 0;
static u16 reg_value_off_thres = 0;

/*****************************************************************************/
/* Prototype                                                                 */
/*****************************************************************************/
static void APDS9900_poll_proximity_sensor(struct work_struct *work);
static int APDS9900_init_input(void);

static irqreturn_t APDS9900_isr(int irq, void* dev_id);

/*****************************************************************************/
/* extern                                                                    */
/*****************************************************************************/
extern int light_create_group(struct input_dev *input);
extern struct attribute_group light_attribute_group;

/*****************************************************************************/
/* function                                                                  */
/*****************************************************************************/








static int APDS9900_power_on(void)
{
    u8 wbuf[2];
    u8 reg;


    PROXIMITY_DPRINTK("START:APDS9900_power_on()\n");
    LIGHT_DPRINTK("START:APDS9900_power_on()\n");



    reg = i2c_smbus_read_byte_data
        (this_client, CMD_REG_REPEATED_BYTE | REG_ADDRESS_ENABLE);


    PROXIMITY_DPRINTK("DEBUG:APDS9900_power_on ENABLE_REG:%x\n", reg);
    LIGHT_DPRINTK("DEBUG:APDS9900_power_on ENABLE_REG:%x\n", reg);



    wbuf[0] = CMD_REG_REPEATED_BYTE | REG_ADDRESS_ENABLE;


    wbuf[1] = REG_ENABLE_PON_CHANGE | reg;


    i2c_master_send(this_client, wbuf, 2);

    PROXIMITY_DPRINTK("END:APDS9900_power_on()\n");
    LIGHT_DPRINTK("END:APDS9900_power_on()\n");

    return RTN_OK;
}









static int APDS9900_power_off(void)
{
    u8 wbuf[2];
    u8 reg;


    PROXIMITY_DPRINTK("START:APDS9900_power_off()\n");
    LIGHT_DPRINTK("START:APDS9900_power_off()\n");


    reg = i2c_smbus_read_byte_data
        (this_client, CMD_REG_REPEATED_BYTE | REG_ADDRESS_ENABLE);


    PROXIMITY_DPRINTK("DEBUG:APDS9900_power_off ENABLE_REG:%x\n", reg);
    LIGHT_DPRINTK("DEBUG:APDS9900_power_off ENABLE_REG:%x\n", reg);



    wbuf[0] = CMD_REG_REPEATED_BYTE | REG_ADDRESS_ENABLE;


    wbuf[1] = ~REG_ENABLE_PON_CHANGE & reg;


    i2c_master_send(this_client, wbuf, 2);

    PROXIMITY_DPRINTK("END:APDS9900_power_off()\n");
    LIGHT_DPRINTK("END:APDS9900_power_off()\n");

    return RTN_OK;
}










static int APDS9900_proximity_change_data(int pdata)
{
    int i, ret_data=proximity_data_tbl.last_data;

    PROXIMITY_DPRINTK("START:%s pdata:%d\n", __func__, pdata);
    
    if(pdata > proximity_thres[0].hi || pdata < proximity_thres[proximity_max_level].lo)
    {
      printk("[prox] %s: invalid param(%d).\n",__func__,pdata);
      return ret_data;
    }
    
    for(i=proximity_data_tbl.last_data;i > 0;i--)
    {
      if(pdata > proximity_thres[i].hi){
        ret_data = i - 1;
      }
      else {
        break;
      }
    }
    
    for(i=proximity_data_tbl.last_data;i < proximity_max_level;i++)
    {
      if(pdata < proximity_thres[i].lo){
        ret_data = i + 1;
      }
      else {
        break;
      }
    }
    
    PROXIMITY_DPRINTK("END:%s ret_data:%d\n", __func__, ret_data);

    return ret_data;
}


static void APDS9900_proximity_change_setting(int level)
{
    u8 wbuf[5] = {0};
    
    if(proximity_log_enable)
    {
      u8 ppcount = 0, pdrive=0;
      
      i2c_smbus_read_i2c_block_data
        (this_client, CMD_REG_REPEATED_BYTE | REG_ADDRESS_PPCOUNT, 1, &ppcount);
      i2c_smbus_read_i2c_block_data
        (this_client, CMD_REG_REPEATED_BYTE | REG_ADDRESS_CONTROL, 1, &pdrive);
      
      PROXIMITY_DPRINTK("DEBUG:%s ppcount:%d, pdrive:%d\n", __func__, ppcount, (pdrive >> 6) & (0x03));
    }



    reg_value_off_thres = proximity_thres[level].lo;
    reg_value_on_thres = proximity_thres[level].hi;

    wbuf[0] = CMD_REG_AUTO_INCREMENT | REG_ADDRESS_PILTL;
    wbuf[1] = reg_value_off_thres & 0x00ff;
    wbuf[2] = reg_value_off_thres >> 8;
    wbuf[3] = reg_value_on_thres & 0x00ff;
    wbuf[4] = reg_value_on_thres >> 8;

    i2c_master_send(this_client, wbuf, 5);

    PROXIMITY_DPRINTK("DEBUG:%s lv:%d hi:%d lo:%d\n",__func__,level,reg_value_on_thres,reg_value_off_thres);
}









static void
APDS9900_poll_proximity_sensor(struct work_struct *work)
{

    u8 rbuf[5] = {0};

    int Prox_data;
    int Prox_change_data;


    PROXIMITY_DPRINTK("START:APDS9900_poll_proximity_sensor()\n");



    i2c_smbus_read_i2c_block_data
        (this_client, CMD_REG_AUTO_INCREMENT | REG_ADDRESS_PDATAL, 2, rbuf);


    Prox_data = (u16) (((rbuf[1]) << 8) | rbuf[0]);


    PROXIMITY_DPRINTK("DEBUG:APDS9900_poll_proximity_sensor PDATAL:%x PDATAH:%x Prox_data:%d\n",
                       rbuf[0],rbuf[1],Prox_data);




    Prox_change_data = APDS9900_proximity_change_data(Prox_data);


    proximity_data_tbl.last_data = Prox_change_data;


    APDS9900_proximity_change_setting(Prox_change_data);


    input_report_abs(input, ABS_X, Prox_change_data * 1000);
    input_sync(input);


    rbuf[0] = CMD_REG_PROX_CLEAR;

    i2c_master_send(this_client, rbuf, 1);

    enable_irq(IRQ_NUM);



    PROXIMITY_DPRINTK("END:APDS9900_poll_proximity_sensor()\n");


}













static irqreturn_t APDS9900_isr(int irq, void* dev_id)
{

    if(irq_dev_id != dev_id)
    {

        PROXIMITY_DPRINTK("DEBUG:GP2AP002A00F_isr not mach device id\n");

        return IRQ_NONE;
    }
    if(gpio_get_value( GPIO_PS_OUT ) != 0)
    {

        PROXIMITY_DPRINTK("DEBUG:not interrupt from GPIO_PS_OUT\n");

        return IRQ_NONE;
    }

    disable_irq_nosync(irq);
    schedule_delayed_work(&apds9900_proximity_work, msecs_to_jiffies(0));

    wake_lock_timeout(&proximity_wake_lock, 2 * HZ);

    return IRQ_HANDLED;
}











































int APDS9900_light_change_lux(int *Lux_data, int *Clear_data, int *IR_data)
{
    int iac,iac_1,iac_2;    /* IR Adjusted Count */   
    unsigned int lpc;      /* Lux per Count */

    unsigned int alsit;    /* ALS integration time */

    unsigned int again;
    u8 reg;


    LIGHT_DPRINTK("START:APDS9900_light_change_lux()\n");



    reg = i2c_smbus_read_byte_data
        (this_client, CMD_REG_REPEATED_BYTE | REG_ADDRESS_ATIME);




    alsit = ((256 - reg) * LUX_CAL_BASE_TIME);


    LIGHT_DPRINTK("DEBUG:APDS9900_light_change_lux ATIME_REG:%x alsit:%d\n",reg, alsit);


    reg = i2c_smbus_read_byte_data
        (this_client, CMD_REG_REPEATED_BYTE | REG_ADDRESS_CONTROL);

    switch(reg & 0x03) {
        case 0x00:
            again = 1;
            break;

        case 0x01:
            again = 8;
            break;

        case 0x02:
            again = 16;
            break;

        case 0x03:
            again = 120;
            break;

        default:

            printk("[LIGHT_SENSOR] ERR:APDS9900_light_change_lux GAIN NOREG\n");

            again = 1;
            break;
    }

    LIGHT_DPRINTK("DEBUG:APDS9900_light_change_lux CTRL_REG:%x again:%d\n",reg, again);






    lpc = LUX_CAL_GA * LUX_CAL_DF / (alsit * again);


    LIGHT_DPRINTK("DEBUG:APDS9900_light_change_lux LPC:%d\n", lpc);






    iac_1 = (*Clear_data) - ((LUX_CAL_COEFFICIENT_B * (*IR_data)) / 1000 );



    LIGHT_DPRINTK("DEBUG:APDS9900_light_change_lux IAC1:%d\n", iac_1);






    iac_2 = ((LUX_CAL_COEFFICIENT_C * (*Clear_data)) - (LUX_CAL_COEFFICIENT_D * (*IR_data))) / 1000;



    LIGHT_DPRINTK("DEBUG:APDS9900_light_change_lux IAC2:%d\n", iac_2);



    if(iac_1 >= iac_2) {
        if(iac_1 >= 0) {
            iac = iac_1;
        }
        else {
            iac = 0;
        }
    }
    else {
        if(iac_2 >= 0) {
            iac = iac_2;
        }
        else {
            iac = 0;
        }
    }

    LIGHT_DPRINTK("DEBUG:APDS9900_light_change_lux IAC:%d\n", iac);



    *Lux_data = (iac * lpc) / 10000;

    LIGHT_DPRINTK("END:APDS9900_light_change_lux()\n");


    return RTN_OK;
}









static int APDS9900_light_measure(int *Clear_data, int *IR_data)
{
    u8 rbuf[2] = {0};


    LIGHT_DPRINTK("START:APDS9900_light_measure()\n");


    i2c_smbus_read_i2c_block_data
        (this_client, CMD_REG_AUTO_INCREMENT | REG_ADDRESS_CDATAL, 2, rbuf);


    *Clear_data = (u16) (((rbuf[1]) << 8) | rbuf[0]);


    LIGHT_DPRINTK("DEBUG:APDS9900_light_measure CLEAR_DATAL:%x CLEAR_DATAH:%x CLEAR_DATA:%d\n",
                   rbuf[0],rbuf[1],*Clear_data);



    i2c_smbus_read_i2c_block_data
        (this_client, CMD_REG_AUTO_INCREMENT | REG_ADDRESS_IRDATAL, 2, rbuf);


    *IR_data = (u16) (((rbuf[1]) << 8) | rbuf[0]);

    LIGHT_DPRINTK("DEBUG:APDS9900_light_measure IR_DATAL:%x IR_DATAH:%x IR_DATA:%d\n",
                   rbuf[0],rbuf[1],*IR_data);
    LIGHT_DPRINTK("END:APDS9900_light_measure()\n");


    return RTN_OK;
}










static int APDS9900_light_threshold_lux(int data)
{
    int i;
    int level = 0;
    const int ll_tbl[] = {
      LIGHT_SENSOR_LUX_VALUE_MIN,
      15,
      100,
      200,
      LIGHT_SENSOR_LUX_VALUE_MAX
    };

    if(data < ll_tbl[0])
    {
      printk("[light] %s: invalid param(%d).\n",__func__,data);
    }

    for(i=1;i < ARRAY_SIZE(ll_tbl);i++)
    {
      if(data > ll_tbl[i]) level++;
    }
    
    return (ll_tbl[level] * 1000);
}

#ifdef APDS9900_WAIT_LUX_CHG











static int APDS9900_wait_lux_chg( int data )
{
    int ret = LIGHT_SENSOR_SEND_WAIT;
    static unsigned int wait_lux_chg = 0;
    
    if(data != LIGHT_SENSOR_LUX_VALUE_MIN) {
        wait_lux_chg = 0;
    }

    if (LIGHT_SENSOR_CHECK_WAITTIME < wait_lux_chg) {
        wait_lux_chg = 0;
    }

    if (0 == wait_lux_chg) {
        ret = LIGHT_SENSOR_SEND_OK;
    }

    wait_lux_chg += atomic_read(&light_data_tbl.delay);

    return ret;
}


#endif









static void
APDS9900_poll_light_sensor(struct work_struct *work)
{
    int Clear_data, IR_data, Lux_data = 0;
    unsigned long delay = delay_to_jiffies(atomic_read(&light_data_tbl.delay));
    int send_data;

    int atime_max_cnt;



    LIGHT_DPRINTK("START:APDS9900_poll_light_sensor()\n");


    APDS9900_light_measure(&Clear_data, &IR_data);



    atime_max_cnt = (((256 - reg_value_atime) * CH0DATA_BASE_COUNT) - 1);

    LIGHT_DPRINTK("DEBUG:CH0DATA MAX COUNT:[%d]\n",atime_max_cnt);


    if(CH0DATA_FULL_SCALE_COUNT < atime_max_cnt) {

        atime_max_cnt = CH0DATA_FULL_SCALE_COUNT;
    }

    if(atime_max_cnt <= Clear_data) {

        Lux_data = LIGHT_SENSOR_LUX_VALUE_MAX + 1;
    }
    else {

        (void)APDS9900_light_change_lux(&Lux_data, &Clear_data, &IR_data);
    }




    LIGHT_DPRINTK("DEBUG:get Lux_data:%d\n",Lux_data);


    send_data = APDS9900_light_threshold_lux(Lux_data);


    LIGHT_DPRINTK("DEBUG:APDS9900_poll_light_sensor send LUX_DATA:%d\n", send_data);





    light_data_tbl.last_data = send_data;




#ifdef APDS9900_WAIT_LUX_CHG
    if(APDS9900_wait_lux_chg(send_data))
#endif

    {


        input_report_abs(input2, ABS_X, light_data_tbl.last_data);

        input_sync(input2);
    }


    schedule_delayed_work(&apds9900_light_work, delay);

    LIGHT_DPRINTK("END:APDS9900_poll_light_sensor()\n");

}









static int
proximity_enable(void)
{

    int ret;
    u8 rbuf[5] = {0};



    PROXIMITY_DPRINTK("START:proximity_enable()\n");

    if (!atomic_cmpxchg(&proximity_data_tbl.enable, 0, 1)) {



		proximity_data_tbl.last_data = proximity_max_level;
		input_report_abs(input, ABS_X, proximity_data_tbl.last_data*1000);


        atomic_set(&proximity_data_tbl.enable, 1);

        APDS9900_power_on();




        rbuf[0] = CMD_REG_AUTO_INCREMENT | REG_ADDRESS_PILTL;
        rbuf[1] = 0x00;
        rbuf[2] = 0x00;
        rbuf[3] = 0x00;
        rbuf[4] = 0x00;

        i2c_master_send(this_client, rbuf, 5);

        ret = request_irq(IRQ_NUM,APDS9900_isr,IRQF_TRIGGER_LOW,PROXIMITY_DEV_NAME,irq_dev_id);
        enable_irq_wake(IRQ_NUM);

    }

    PROXIMITY_DPRINTK("END:proximity_enable()\n");


    return RTN_OK;
}









static int
proximity_disable(void)
{

    PROXIMITY_DPRINTK("START:proximity_disable()\n");

    if (atomic_cmpxchg(&proximity_data_tbl.enable, 1, 0)) {
        if (!atomic_read(&light_data_tbl.enable)) {

            APDS9900_power_off();
        }
        atomic_set(&proximity_data_tbl.enable, 0);

        disable_irq_wake(IRQ_NUM);


        free_irq(IRQ_NUM,irq_dev_id);

    }

    PROXIMITY_DPRINTK("END:proximity_disable()\n");


    return RTN_OK;
}












static ssize_t
proximity_enable_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    return sprintf(buf, "%d\n", atomic_read(&proximity_data_tbl.enable));
}














static ssize_t
proximity_enable_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf,
        size_t count)
{
    int value;


    PROXIMITY_DPRINTK("START:proximity_enable_store()\n");

    value = simple_strtoul(buf, NULL, 10);
    if (value != 0 && value != 1) {

        printk("[PROXIMITY_SENSOR] ERROR:proximity_enable_store value:%d\n", value);

        return count;
    }

    if (value) {
        proximity_enable();
    }
    else {
        proximity_disable();
    }

    PROXIMITY_DPRINTK("END:proximity_enable_store()\n");

    return count;
}












static ssize_t
proximity_delay_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    return sprintf(buf, "%d\n", atomic_read(&proximity_data_tbl.delay));
}












static void APDS9900_proximity_set_delay(struct device *dev, unsigned int delay)
{

    PROXIMITY_DPRINTK("START:APDS9900_proximity_set_delay() delay:%d\n",delay);

    atomic_set(&proximity_data_tbl.delay, delay);

    PROXIMITY_DPRINTK("END:APDS9900_proximity_set_delay()\n");

}














static ssize_t
proximity_delay_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf,
        size_t count)
{
    unsigned long delay;

    PROXIMITY_DPRINTK("START:proximity_delay_store()\n");

    delay = simple_strtoul(buf, NULL, 10);
    APDS9900_proximity_set_delay(dev, delay);

    PROXIMITY_DPRINTK("END:proximity_delay_store()\n");


    return count;
}














static ssize_t
proximity_wake_store(struct device *dev,
                                struct device_attribute *attr,
                                const char *buf, size_t count)
{
    struct input_dev *input_dat = to_input_dev(dev);
    static atomic_t serial = ATOMIC_INIT(0);

    PROXIMITY_DPRINTK("START:proximity_wake_store()\n");

    input_report_abs(input_dat, ABS_MISC, atomic_inc_return(&serial));

    PROXIMITY_DPRINTK("END:proximity_wake_store()\n");


    return count;
}












static ssize_t
proximity_data_show(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", proximity_data_tbl.last_data * 1000);
}



















static ssize_t proximity_dbglog_show(struct device *dev,
                                      struct device_attribute *attr, char *buf)
{
    if(proximity_log_enable)
    {
        proximity_log_enable = false;
        printk("[PROXIMITY_SENSOR] DEBUG: DEBUG_OFF\n");
    }
    else
    {
        proximity_log_enable = true;
        printk("[PROXIMITY_SENSOR] DEBUG: DEBUG_ON\n");
    }

    return sprintf(buf, "%d\n", proximity_log_enable);
}



static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP,
                    proximity_enable_show, proximity_enable_store);
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP,
                    proximity_delay_show, proximity_delay_store);
static DEVICE_ATTR(wake, S_IWUSR|S_IWGRP, NULL, proximity_wake_store);
static DEVICE_ATTR(data, S_IRUGO, proximity_data_show, NULL);

static DEVICE_ATTR(dbglog, S_IRUGO, proximity_dbglog_show, NULL);


static struct attribute *proximity_attributes[] = {
    &dev_attr_enable.attr,
    &dev_attr_delay.attr,
    &dev_attr_wake.attr,
    &dev_attr_data.attr,

    &dev_attr_dbglog.attr,

    NULL
};

static struct attribute_group proximity_attribute_group = {
    .attrs = proximity_attributes
};










unsigned int light_enable_cal_delay(void)
{
    unsigned int cal_delay = 0;
    unsigned int delay_decimal = 0;
    unsigned int ret_delay = 0;
    unsigned int atime_ms = 0;
    unsigned int ptime_ms = 0;
    unsigned int wtime_ms = 0;


    LIGHT_DPRINTK("START:light_enable_cal_delay atime:[%x] ptime:[%x] wtime:[%x] config:[%x]\n",
        reg_value_atime,reg_value_ptime,reg_value_wtime,reg_value_config);


    atime_ms = ((256 - reg_value_atime) * DELAY_CAL_BASE_TIME);
    ptime_ms = ((256 - reg_value_ptime) * DELAY_CAL_BASE_TIME);
    wtime_ms = ((256 - reg_value_wtime) * DELAY_CAL_BASE_TIME);


    if((reg_value_config >> 1) & 0x01){
        wtime_ms *= 12;
    }

    LIGHT_DPRINTK("DEBUG:light_enable_cal_delay atime_ms:[%d] ptime_ms:[%d] wtime_ms:[%d]\n",
        atime_ms,ptime_ms,wtime_ms);



    cal_delay = ((atime_ms + ptime_ms + wtime_ms + ALS_DELAY) * DELAY_CAL_MARGIN);


    ret_delay = cal_delay / 10000;

    delay_decimal = cal_delay % 10000;

    if(delay_decimal > 0){
        ret_delay += 1;
     }

    LIGHT_DPRINTK("END:light_enable_cal_delay ret_delay:[%d]\n",ret_delay);

    return(ret_delay);
}










static int
light_enable(void)
{

    unsigned int  delay;


    LIGHT_DPRINTK("START:light_enable()\n");

    if (!atomic_cmpxchg(&light_data_tbl.enable, 0, 1)) {

        APDS9900_power_on();

        atomic_set(&light_data_tbl.enable, 1);



        delay = light_enable_cal_delay();




        schedule_delayed_work(&apds9900_light_work, msecs_to_jiffies(delay));


    }

    LIGHT_DPRINTK("END:light_enable()\n");


    return RTN_OK;
}









static int
light_disable(void)
{

    LIGHT_DPRINTK("START:light_disable()\n");

    if(atomic_cmpxchg(&light_data_tbl.enable, 1, 0)) {
        if(!atomic_read(&proximity_data_tbl.enable)) {

            APDS9900_power_off();
        }
        atomic_set(&light_data_tbl.enable, 0);
        cancel_delayed_work_sync(&apds9900_light_work);






        input_report_abs(input2, ABS_X, light_data_tbl.last_data);
        input_sync(input2);

    }

    LIGHT_DPRINTK("END:light_disable()\n");


    return RTN_OK;
}












ssize_t
light_enable_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    return sprintf(buf, "%d\n", atomic_read(&light_data_tbl.enable));
}














ssize_t
light_enable_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf,
        size_t count)
{
    int value;

    LIGHT_DPRINTK("START:light_enable_store()\n");

    value = simple_strtoul(buf, NULL, 10);
    if (value != 0 && value != 1) {

        printk("[LIGHT_SENSOR] ERROR:light_enable_store value:%d\n", value);

        return count;
    }

    if (value) {
        light_enable();
    }
    else {
        light_disable();
    }

    LIGHT_DPRINTK("END:light_enable_store()\n");

    return count;
}












ssize_t
light_delay_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    return sprintf(buf, "%d\n", atomic_read(&light_data_tbl.delay));
}












static void APDS9900_light_set_delay(struct device *dev, unsigned int delay)
{

    LIGHT_DPRINTK("START:APDS9900_light_set_delay() delay:%d\n",delay);

    atomic_set(&light_data_tbl.delay, delay);

    if (atomic_read(&light_data_tbl.enable)) {
        cancel_delayed_work_sync(&apds9900_light_work);
        schedule_delayed_work(&apds9900_light_work, delay_to_jiffies(delay));
    }

    LIGHT_DPRINTK("END:APDS9900_light_set_delay()\n");

}














ssize_t
light_delay_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf,
        size_t count)
{
    unsigned int delay;

    LIGHT_DPRINTK("START:light_delay_store()\n");

    delay = simple_strtoul(buf, NULL, 10);
    APDS9900_light_set_delay(dev, delay);

    LIGHT_DPRINTK("END:light_delay_store()\n");

    return count;
}














ssize_t
light_wake_store(struct device *dev,
                                struct device_attribute *attr,
                                const char *buf, size_t count)
{
    struct input_dev *input_dat = to_input_dev(dev);
    static atomic_t serial = ATOMIC_INIT(0);

    LIGHT_DPRINTK("START:light_wake_store()\n");

    input_report_abs(input_dat, ABS_MISC, atomic_inc_return(&serial));

    LIGHT_DPRINTK("END:light_wake_store()\n");

    return count;
}












ssize_t
light_data_show(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", light_data_tbl.last_data);
}



















ssize_t light_dbglog_show(struct device *dev,
                          struct device_attribute *attr, char *buf)
{
    if(light_log_enable)
    {
        light_log_enable = false;
        printk("[LIGHT_SENSOR] DEBUG: DEBUG_OFF\n");
    }
    else
    {
        light_log_enable = true;
        printk("[LIGHT_SENSOR] DEBUG: DEBUG_ON\n");
    }

    return sprintf(buf, "%d\n", light_log_enable);
}













static int
APDS9900_init_input_proximity(int *sysfs_input_created, int *input_registered)
{
    int err;

    printk("[APDS9900] START:APDS9900_init_input_proximity()\n");


    input = input_allocate_device();

    if (input == NULL) {
        err = -ENOMEM;

        printk("[APDS9900] ERROR:input_allocate_device(input) err:%d\n", err);

        return err;
    }

    input->name = "proximity";
    input->id.bustype = BUS_I2C;

    input_set_capability(input, EV_ABS, ABS_MISC);
    input_set_abs_params(input, ABS_X, 0, 1, 0, 0);
    input_set_drvdata(input, NULL);
    err = input_register_device(input);

    if(err) {

        printk("[APDS9900] ERROR:input_register_device(input) name:[%s] err:%d\n", input->name, err);

        return err;
    }

    *input_registered = 1;

    INIT_DELAYED_WORK(&apds9900_proximity_work,APDS9900_poll_proximity_sensor);

    atomic_set(&proximity_data_tbl.enable, 0);
    atomic_set(&proximity_data_tbl.delay, msecs_to_jiffies(PROXIMITY_SENSOR_DEFAULT_DELAY));
    proximity_data_tbl.last_data = 0;

    err = sysfs_create_group(&input->dev.kobj,&proximity_attribute_group);

    if(err) {

        printk("[APDS9900] ERROR:sysfs_create_group failed name:[%s] err:%d\n", input->name, err);

        return err;
    }

    *sysfs_input_created = 1;

    printk("[APDS9900] END:APDS9900_init_input_proximity()\n");

    return RTN_OK;
}











static int
APDS9900_init_input_light(int *sysfs_input2_created, int *input2_registered)
{
    int err;

    printk("[APDS9900] START:APDS9900_init_input_light()\n");


    input2 = input_allocate_device();

    if (input2 == NULL) {
        err = -ENOMEM;

        printk("[APDS9900] ERROR:input_allocate_device(input2) err:%d\n", err);

        return err;
    }

    input2->name = "light";
    input2->id.bustype = BUS_I2C;

    input_set_capability(input2, EV_ABS, ABS_MISC);
    input_set_abs_params(input2, ABS_X, 0, 1, 0, 0);
    input_set_drvdata(input2, NULL);
    err = input_register_device(input2);

    if(err) {

        printk("[APDS9900] ERROR:input_register_device(input2) name:[%s] err:%d\n", input2->name, err);

        return err;
    }

    *input2_registered = 1;


    INIT_DELAYED_WORK(&apds9900_light_work,APDS9900_poll_light_sensor);

    atomic_set(&light_data_tbl.enable, 0);
    atomic_set(&light_data_tbl.delay, msecs_to_jiffies(LIGHT_SENSOR_DEFAULT_DELAY));
    light_data_tbl.last_data = 0;

    err = light_create_group(input2);

    if(err) {

        printk("[APDS9900] ERROR:light_create_group failed name:[%s] err:%d\n", input2->name, err);

        return err;
    }

    *sysfs_input2_created = 1;

    printk("[APDS9900] END:APDS9900_init_input_light()\n");

    return 0;
}










static int
APDS9900_init_input(void)
{
    int err;
    int sysfs_input_created = 0, sysfs_input2_created = 0;
    int input_registered = 0, input2_registered = 0;

    printk("[APDS9900] START:APDS9900_init_input()\n");


    err = APDS9900_init_input_proximity(&sysfs_input_created, &input_registered);

    if(err) {

        printk("[APDS9900] ERROR:APDS9900_init_input_proximity\n");

        if (input != NULL) {
            if (sysfs_input_created) {
                sysfs_remove_group(&input->dev.kobj, &proximity_attribute_group);
            }
            if (input_registered) {
                input_unregister_device(input);
            }
            else {
                input_free_device(input);
            }
        }

        printk("[APDS9900] END:APDS9900_init_input() NG_RTN\n");

        return err;
    }


    err = APDS9900_init_input_light(&sysfs_input2_created, &input2_registered);

    if(err) {

        printk("[APDS9900] ERROR:APDS9900_init_input_light\n");

        if (input2 != NULL) {
            if (sysfs_input2_created) {
                sysfs_remove_group(&input2->dev.kobj, &light_attribute_group);
            }
            if (input2_registered) {
                input_unregister_device(input2);
            }
            else {
                input_free_device(input2);
            }
        }

        printk("[APDS9900] END:APDS9900_init_input() NG_RTN\n");

        return err;
    }

    printk("[APDS9900] END:APDS9900_init_input() OK_RTN\n");

    return RTN_OK;
}









static int APDS9900_hw_init(void)
{
    u8 wbuf[2];
    int type = TYPE_DEFAULT;
    

    printk("[APDS9900] START:APDS9900_hw_init()\n");


    APDS9900_power_off();





    reg_value_atime = REG_ATIME_INIT_VALUE;
    reg_value_ptime = REG_PTIME_INIT_VALUE;
    reg_value_wtime = REG_WTIME_INIT_VALUE;
    reg_value_config = REG_CONFIG_INIT_VALUE;



#ifndef CONFIG_TARGET_PROTOTYPE_WS2
    if(HS_HW_ID == HS_HW_ID_SFM)
    {
        type = TYPE_SFN;
    }
#endif /* CONFIG_TARGET_PROTOTYPE_WS2 */

    if(HS_HW_ID >= HS_HW_ID_LAB)
    {
        type = TYPE_LAB;
    }

    if(type==TYPE_SFN)
    {
        reg_value_control   = 0x20 | SFN_REG_CONTROL_PDRIVE_INIT_VALUE;
        reg_value_ppcount   = SFN_REG_PPCOUNT_INIT_VALUE;
        proximity_thres     = (struct proximity_threshold *)sfn_proximity_thres_tbl;
        proximity_max_level = ARRAY_SIZE(sfn_proximity_thres_tbl)-1;
    }
    else if(type==TYPE_LAB)
    {
        reg_value_control   = 0x20 | LAB_REG_CONTROL_PDRIVE_INIT_VALUE;
        reg_value_ppcount   = LAB_REG_PPCOUNT_INIT_VALUE;
        proximity_thres     = (struct proximity_threshold *)lab_proximity_thres_tbl;
        proximity_max_level = ARRAY_SIZE(lab_proximity_thres_tbl)-1;
    }
    else
    {
        reg_value_control   = 0x20 | REG_CONTROL_PDRIVE_INIT_VALUE;
        reg_value_ppcount   = REG_PPCOUNT_INIT_VALUE;
        proximity_thres     = (struct proximity_threshold *)proximity_thres_tbl;
        proximity_max_level = ARRAY_SIZE(proximity_thres_tbl)-1;
    }


    wbuf[0] = CMD_REG_REPEATED_BYTE | REG_ADDRESS_ATIME;


    wbuf[1] = reg_value_atime;



    i2c_master_send(this_client, wbuf, 2);


    wbuf[0] = CMD_REG_REPEATED_BYTE | REG_ADDRESS_PTIME;

    wbuf[1] = reg_value_ptime;


    i2c_master_send(this_client, wbuf, 2);


    wbuf[0] = CMD_REG_REPEATED_BYTE | REG_ADDRESS_WTIME;

    wbuf[1] = reg_value_wtime;


    i2c_master_send(this_client, wbuf, 2);


    wbuf[0] = CMD_REG_REPEATED_BYTE | REG_ADDRESS_PPCOUNT;

    wbuf[1] = reg_value_ppcount;


    i2c_master_send(this_client, wbuf, 2);


    wbuf[0] = CMD_REG_REPEATED_BYTE | REG_ADDRESS_CONTROL;
    wbuf[1] = reg_value_control;

    i2c_master_send(this_client, wbuf, 2);





    wbuf[0] = CMD_REG_REPEATED_BYTE | REG_ADDRESS_PERS;
    wbuf[1] = 0x10;

    i2c_master_send(this_client, wbuf, 2);



    wbuf[0] = CMD_REG_REPEATED_BYTE | REG_ADDRESS_ENABLE;


    wbuf[1] = 0x2e;


    i2c_master_send(this_client, wbuf, 2);

    printk("[APDS9900] END:APDS9900_hw_init()\n");


    return RTN_OK;
}


static int APDS9900_open(struct inode *inode, struct file *file)
{
    return nonseekable_open(inode, file);
}

static int APDS9900_release(struct inode *inode, struct file *file)
{
    return 0;
}

static struct file_operations APDS9900_fops = {
    .owner = THIS_MODULE,
    .open  = APDS9900_open,
    .release = APDS9900_release,
};

static struct miscdevice APDS9900_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name  = PROXIMITY_DEV_NAME,
    .fops = &APDS9900_fops,
};











static int
APDS9900_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int i_Ret;

    printk("[APDS9900] START:APDS9900_probe()\n");


    gpio_tlmm_config
        (GPIO_CFG(GPIO_PS_OUT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

    i2c_set_clientdata(client, NULL);
    this_client = client;


    i_Ret = misc_register(&APDS9900_device);
    if(i_Ret) {

        printk("[APDS9900] ERROR:misc_register i_Ret:%d\n", i_Ret);
        printk("[APDS9900] END:APDS9900_probe() NG_RTN\n");

        return i_Ret;
    }


    APDS9900_hw_init();


    APDS9900_init_input();

    wake_lock_init(&proximity_wake_lock, WAKE_LOCK_SUSPEND, "proximity");


    printk("[APDS9900] END:APDS9900_probe() OK_RTN\n");

    return RTN_OK;
}









static int
APDS9900_remove(struct i2c_client *client)
{

    printk("[APDS9900] START:APDS9900_remove()\n");

    proximity_disable();
    light_disable();

    sysfs_remove_group(&input->dev.kobj, &proximity_attribute_group);

    input_unregister_device(input);
    input_free_device(input);

    printk("[APDS9900] END:APDS9900_remove()\n");

    return RTN_OK;
}











static int
APDS9900_suspend(struct i2c_client *client, pm_message_t mesg)
{

    PROXIMITY_DPRINTK("START:APDS9900_suspend()\n");
    LIGHT_DPRINTK("START:APDS9900_suspend()\n");
    PROXIMITY_DPRINTK("END:APDS9900_suspend()\n");
    LIGHT_DPRINTK("END:APDS9900_suspend()\n");

    return RTN_OK;
}









static int
APDS9900_resume(struct i2c_client *client)
{

    PROXIMITY_DPRINTK("START:APDS9900_resume()\n");
    LIGHT_DPRINTK("START:APDS9900_resume()\n");
    PROXIMITY_DPRINTK("END:APDS9900_resume()\n");
    LIGHT_DPRINTK("END:APDS9900_resume()\n");

    return RTN_OK;
}


static struct i2c_device_id APDS9900_idtable[] = {
    {PROXIMITY_DEV_NAME, 0},
    {}
};
MODULE_DEVICE_TABLE(i2c, APDS9900_idtable);

static struct i2c_driver APDS9900_i2c_driver = {
    .driver = {
        .name       = PROXIMITY_DEV_NAME,
        .owner      = THIS_MODULE,
    },

    .id_table       = APDS9900_idtable,
    .probe          = APDS9900_probe,
    .remove         = APDS9900_remove,
    .suspend        = APDS9900_suspend,
    .resume         = APDS9900_resume,
};








static int __init
APDS9900_init(void)
{
    hs_vreg_ctl("gp4", ON_CMD, 2600000, 2600000);
#ifdef CONFIG_TARGET_PROTOTYPE_WS0
    hs_vreg_ctl("lvsw1", ON_CMD, 1800000, 1800000);
#endif /* CONFIG_TARGET_PROTOTYPE_WS0 */
    mdelay(5);


    printk("[APDS9900] START:APDS9900_init()\n");
    printk("[APDS9900] END:APDS9900_init()\n");

    return i2c_add_driver(&APDS9900_i2c_driver);
}








static void __exit
APDS9900_term(void)
{

    printk("[APDS9900] START:APDS9900_term()\n");
    i2c_del_driver(&APDS9900_i2c_driver);
    printk("[APDS9900] END:APDS9900_term()\n");

}

module_init(APDS9900_init);
module_exit(APDS9900_term);

MODULE_AUTHOR("Kyocera");
MODULE_DESCRIPTION("Kyocera proximity and Light Sensor Driver");
MODULE_LICENSE( "GPL" );
MODULE_VERSION("1.0.0");
